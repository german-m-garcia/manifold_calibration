#ifndef SOLVER_H
#define SOLVER_H

#include "kinematics.h"
#include <Eigen/Geometry>
#include <vector>

namespace ManifoldCalibration
{

    struct DataSample
    {

        DataSample(double time);

        double time;
    };

    struct KinematicsSample : DataSample
    {
        KinematicsSample(int deltaTicsLeft, int deltaTicsRight, double time);

        int deltaTicsLeft;
        int deltaTicsRight;
    };

    struct SensorSample : DataSample
    {
        SensorSample(const Eigen::Isometry2d &tf, double time);
        //! Relative transformation in the sensor's local coordinate system
        Eigen::Isometry2d relativeTransf;
    };

    struct DatasetSample
    {
        bool skip() const;
        int accTicsL() const;
        int accTicsR() const;
        SensorSample sensorSample;
        std::vector<KinematicsSample> kinematicsSamples;
    };

    class Dataset2d
    {
    public:
        Dataset2d();
        void addSample(const KinematicsSample &);
        void addSample(const SensorSample &);
        size_t size() const;
        DatasetSample sample(size_t batch, size_t i) const;
        DatasetSample sample(size_t i) const;

        size_t nBatches() const;
        size_t batchSize() const;
        void debuginfo() const;
        void sort();

    private:
        std::vector<KinematicsSample> _kinematicsSamples;
        std::vector<SensorSample> _sensorSamples;

        size_t _batchSize{20};
    };

    class LidarDiffDriveParameters
    {
    public:
        LidarDiffDriveParameters(const Eigen::Vector3d &initialKinematicsParameters, int ticsPerRevolution, const Eigen::Vector3d &initialSensorPoseParameters);
        double kl() const;
        double kr() const;
        double kb() const;
        LidarDiffDriveParameters addPerturbation(const Eigen::VectorXd &perturbation) const;
        Eigen::Vector3d kinematicsParameters() const;
        Eigen::Vector3d sensorParameters() const;

        Eigen::Vector3d sensorPerturbation(const Eigen::VectorXd &perturbation) const;
        Eigen::Vector3d kinematicsPerturbation(const Eigen::VectorXd &perturbation) const;

        void print() const;

    private:
        Eigen::VectorXd _k;
        int _ticsPerRevolution;
    };

    /*
        X: parameters to be estimated
        Dx: Delta of x
        Oi: Omega of measurement i
        e: error function
        h: prediction function
        (+) and (-): composition operations on the manifold

        H <- 0
        b <- 0
        for each measurement i:
            ei := hi(X*) (-) Z[i]
            Ji := d{e( X* (+) Dx)}/ dDx at Dx = 0

            H := H + Ji^T Oi Ji
            b := b + Ji^T Oi ei

        Dx := solve( H Dx = -b)
        X* := X* + Dx
    */

    // Most basic solver. We are calibrating the extrinsic pose of a 2d sensor (a 2d Lidar)
    // with respect to the base_link of the mobile base, as well as the differential drive parameters
    class Solver2d
    {
    public:
        Solver2d(const DifferentialDriveKinematics &differentialKinematics, const Eigen::Vector3d &initialKinematicsParameters, const Eigen::Vector3d &initialSensorPose, int iterations);
        void addKinematicsSample(const KinematicsSample &);
        void addSensorSample(const SensorSample &);

        void solve();

    private:
        Eigen::Isometry2d odometryRelativeIncrement(const LidarDiffDriveParameters &k, const DatasetSample &sample) const;
        //! Computes the predicted displacement of the sensor frame of reference given the motion of the base according to wheel odometry and the sensor pose
        //! ẑ = h(ks) in SE(2)
        Eigen::Isometry2d h(const Eigen::Isometry2d &base_relative_tf, const Eigen::Isometry2d &sensor_pose) const;

        //! The error is computed in the sensor frame of reference
        //! e(k) = f(ko, u) + ks - z in R3
        Eigen::Vector3d error(const Eigen::Isometry2d &prediction, const Eigen::Isometry2d &measurement) const;

        Eigen::MatrixXd jacobian(const DatasetSample &sample, const Eigen::Isometry2d &sensor_relative_increment) const;

        Dataset2d _dataset;
        DifferentialDriveKinematics _differentialKinematics;
        LidarDiffDriveParameters _k;
        int _iterations = 20;
    };
}

#endif