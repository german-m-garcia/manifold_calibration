#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iomanip>

#include <mk_odometry/Encoder.h>
#include "solver.h"
#include "kinematics.h"
#include "conversions.h"

// using namespace sensor_msgs;

std::unique_ptr<ManifoldCalibration::Solver2d> solver2d;
std::vector<std::pair<int, ros::Time>> ticTimeL, ticTimeR;
std::vector<std::pair<Eigen::Isometry2d, ros::Time>> laserTransfTime;

std::string laserFrameId = "laser";

void callbackTicsL(const mk_odometry::Encoder::ConstPtr &msg)
{
    ticTimeL.push_back(std::make_pair(msg->ticks, msg->header.stamp));
    // ROS_INFO_STREAM("tics left: " << msg->ticks << " time is " << msg->header.stamp << " ticTimeL.size()=" << ticTimeL.size());
}

void callbackTicsR(const mk_odometry::Encoder::ConstPtr &msg)
{
    ticTimeR.push_back(std::make_pair(msg->ticks, msg->header.stamp));
    // ROS_INFO_STREAM("tics right: " << msg->ticks << " time is " << msg->header.stamp << " ticTimeL.size()=" << ticTimeL.size());
}

void callbackTf(const tf2_msgs::TFMessage::ConstPtr &msg)
{
    for (auto tf : msg->transforms)
    {
        if (tf.child_frame_id == laserFrameId)
        {
            double yaw, pitch, roll;
            tf.transform.translation.x, tf.transform.translation.y;
            tf2::getEulerYPR(tf.transform.rotation, yaw, pitch, roll);

            Eigen::Vector3d vectorTf(tf.transform.translation.x, tf.transform.translation.y, yaw);
            // ROS_INFO_STREAM(" r p y = " << roll << " " << pitch << " " << yaw << " vectorTf: " << vectorTf);

            Eigen::Isometry2d absoluteTransf = ManifoldCalibration::fromVector(vectorTf);
            laserTransfTime.push_back(std::make_pair(absoluteTransf, tf.header.stamp));
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manifold_calibration_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    std::setprecision(std::numeric_limits<double>::digits10 + 1);
    std::string leftEncoderTopic("/makeblock/lwheel");
    std::string rightEncoderTopic("/makeblock/rwheel");
    nh.param<std::string>("left_encoder_topic", leftEncoderTopic, "/makeblock/lwheel");
    nh.param<std::string>("right_encoder_topic", rightEncoderTopic, "/makeblock/rwheel");

    double radiusL = 0.035;
    double radiusR = 0.035;
    int ticsPerRevolution = 360;
    double baseline = 0.10;
    nh.param<double>("radius_left", radiusL, 0.035);
    nh.param<double>("radius_right", radiusR, 0.035);
    nh.param<int>("tics_per_revolution", ticsPerRevolution, 360);
    nh.param<double>("baseline", baseline, 0.10);

    rosbag::Bag bag("/home/tomatito/ws/src/calibration/manifold_calibration/data/2D/seq2-2d.bag");
    rosbag::View view(bag, rosbag::TopicQuery({"tf", leftEncoderTopic, rightEncoderTopic}));
    BOOST_FOREACH (rosbag::MessageInstance const m, view)
    {
        if (m.getTopic() == leftEncoderTopic)
        {
            if (mk_odometry::Encoder::ConstPtr msg = m.instantiate<mk_odometry::Encoder>())
                callbackTicsL(msg);
        }
        else if (m.getTopic() == rightEncoderTopic)
        {
            if (mk_odometry::Encoder::ConstPtr msg = m.instantiate<mk_odometry::Encoder>())
                callbackTicsR(msg);
        }
        else if (m.getTopic() == "tf")
        {
            if (tf2_msgs::TFMessage::ConstPtr msg = m.instantiate<tf2_msgs::TFMessage>())
                callbackTf(msg);
        }
    }
    bag.close();
    std::cout << "Bagfile closed" << std::endl;

    ManifoldCalibration::DifferentialDriveKinematics kinematics(ticsPerRevolution);
    //                               ( x y theta )
    double sensor_x = 0., sensor_y = 0., sensor_theta = 0.;
    nh.param<double>("sensor_x", sensor_x, 0.0);
    nh.param<double>("sensor_y", sensor_y, 0.0);
    nh.param<double>("sensor_theta", sensor_theta, 0.0);
    Eigen::Vector3d initialSensorPose(sensor_x, sensor_y, sensor_theta);
    Eigen::Vector3d initialKinematicsParameters(radiusL, radiusR, baseline);
    ROS_INFO_STREAM("initialKinematicsParameters=" << initialKinematicsParameters);

    int iterations = 20;
    nh.param<int>("iterations", iterations, 20);
    ROS_INFO_STREAM("iterations=" << iterations);
    solver2d = std::make_unique<ManifoldCalibration::Solver2d>(kinematics, initialKinematicsParameters, initialSensorPose, iterations);

    ROS_INFO_STREAM(" ticTimeL.size()=" << ticTimeL.size());
    ROS_INFO_STREAM(" ticTimeR.size()=" << ticTimeR.size());
    ROS_INFO_STREAM(" laserTransfTime.size()=" << laserTransfTime.size());

    size_t kinematicsSize = ticTimeL.size() < ticTimeR.size() ? ticTimeL.size() : ticTimeR.size();
    for (size_t i = 1; i < kinematicsSize; i++)
    {
        int ticsL = ticTimeL[i].first - ticTimeL[i - 1].first;
        int ticsR = ticTimeR[i].first - ticTimeR[i - 1].first;
        double timestamp = ticTimeL[i].second.toSec();
        ManifoldCalibration::KinematicsSample kinematicsSample(ticsL, ticsR, timestamp);
        solver2d->addKinematicsSample(kinematicsSample);
    }
    Eigen::Isometry2d globalTf = Eigen::Isometry2d::Identity();
    for (size_t i = 1; i < laserTransfTime.size(); i++)
    {
        ManifoldCalibration::SensorSample currentSensorSample(laserTransfTime[i].first, laserTransfTime[i].second.toSec());
        ManifoldCalibration::SensorSample prevSensorSample(laserTransfTime[i - 1].first, laserTransfTime[i - 1].second.toSec());
        Eigen::Isometry2d relativeTf = currentSensorSample.relativeTransf * prevSensorSample.relativeTransf.inverse();
        ManifoldCalibration::SensorSample sensorSample(relativeTf, currentSensorSample.time);
        // std::cout << "time " << currentSensorSample.time << " relativeTf in sensor coordinates= " << ManifoldCalibration::toVector(relativeTf) << std::endl;
        solver2d->addSensorSample(sensorSample);

        globalTf = relativeTf * globalTf;
    }

    // check that the sensor relative tfs are correct
    ManifoldCalibration::SensorSample lastSensorSample(laserTransfTime[laserTransfTime.size() - 1].first, laserTransfTime[laserTransfTime.size() - 1].second.toSec());
    std::cout << "_____________________________________________________________________" << std::endl;
    std::cout << " lastSensorSample tf: " << ManifoldCalibration::toVector(lastSensorSample.relativeTransf) << std::endl;
    std::cout << " globalTf: " << ManifoldCalibration::toVector(globalTf) << std::endl;
    std::cout << "_____________________________________________________________________" << std::endl;

    solver2d->solve();

    ros::shutdown();
}
