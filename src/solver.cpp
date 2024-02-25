#include "solver.h"
#include "conversions.h"
#include <iostream>
#include <iomanip>

namespace ManifoldCalibration
{
    DataSample::DataSample(double time_) : time(time_)
    {
    }

    KinematicsSample::KinematicsSample(int deltaTicsLeft_, int deltaTicsRight_, double time) : deltaTicsLeft(deltaTicsLeft_), deltaTicsRight(deltaTicsRight_), DataSample(time)
    {
    }

    SensorSample::SensorSample(const Eigen::Isometry2d &tf, double time) : relativeTransf(tf), DataSample(time)
    {
    }
    // ----- DatasetSample
    bool DatasetSample::skip() const
    {
        for (const auto &kinematicsSample : kinematicsSamples)
        {
            if (kinematicsSample.deltaTicsLeft > 0 || kinematicsSample.deltaTicsRight > 0)
                return false;
        }
        return true;
    }

    int DatasetSample::accTicsL() const
    {
        int result = 0;
        for (const auto &kinematicsSample : kinematicsSamples)
            result += kinematicsSample.deltaTicsLeft;
        return result;
    }
    int DatasetSample::accTicsR() const
    {
        int result = 0;
        for (const auto &kinematicsSample : kinematicsSamples)
            result += kinematicsSample.deltaTicsRight;
        return result;
    }

    // ----- Dataset2d

    Dataset2d::Dataset2d()
    {
    }

    void Dataset2d::addSample(const KinematicsSample &sample)
    {
        _kinematicsSamples.push_back(sample);
    }
    void Dataset2d::addSample(const SensorSample &sample)
    {
        _sensorSamples.push_back(sample);
    }

    size_t Dataset2d::size() const
    {
        size_t sensorSamples = _sensorSamples.size();
        size_t kinematicSamples = _kinematicsSamples.size();
        return sensorSamples < kinematicSamples ? sensorSamples : kinematicSamples;
    }

    struct TimeInterval
    {
        bool contains(double t) const
        {
            return t0 < t && t <= tf;
        }
        double t0;
        double tf;
    };

    DatasetSample
    Dataset2d::sample(size_t batch, size_t i) const
    {
        // look for the index-th datasample in the sensor vector
        size_t index = batch * batchSize() + i;
        std::cout << " -----------------------------------------------------------Dataset2d::sample indexing batch #" << batch << " i=" << i << " result index=" << index << std::endl;
        return sample(index);
    }

    DatasetSample Dataset2d::sample(size_t index) const
    {
        SensorSample sensor_sample = _sensorSamples[index];
        std::vector<KinematicsSample> kinematicsSamples;
        // look for all the kinematics samples that happened in the time interval
        double interval = 0.;
        if (index == 0)
            interval = _sensorSamples[index + 1].time - sensor_sample.time;
        else
            interval = sensor_sample.time - _sensorSamples[index - 1].time;

        TimeInterval timeInterval{sensor_sample.time - interval, sensor_sample.time};

        for (const auto &kinematicsSample : _kinematicsSamples)
        {
            if (timeInterval.contains(kinematicsSample.time))
                kinematicsSamples.push_back(kinematicsSample);
        }
        return DatasetSample{sensor_sample, kinematicsSamples};
    }

    size_t Dataset2d::nBatches() const
    {
        std::cout << "Dataset2d::nBatches: size()=" << size() << " _batchSize=" << _batchSize << " std::ceil(size() / _batchSize)=" << std::ceil(1. * size() / _batchSize) << std::endl;
        return std::ceil(1. * size() / _batchSize);
    }

    size_t Dataset2d::batchSize() const
    {
        return _batchSize;
    }

    void Dataset2d::debuginfo() const
    {
        std::cout << "sensor samples: " << _sensorSamples.size() << std::endl;
        std::cout << "kinematic samples: " << _kinematicsSamples.size() << std::endl;
        for (int i = 0; i < 20; i++)
        {
            std::cout << std::setprecision(15) << "sensor sample #" << i << " time= " << _sensorSamples[i].time << std::endl;
        }
        for (int i = 0; i < 20; i++)
        {
            std::cout << std::setprecision(15) << "kinematics sample #" << i << " time= " << _kinematicsSamples[i].time << std::endl;
        }
    }

    void Dataset2d::sort()
    {
        std::sort(_sensorSamples.begin(), _sensorSamples.end(), [](const SensorSample &s1, const SensorSample &s2)
                  { return s1.time < s2.time; });
        std::sort(_kinematicsSamples.begin(), _kinematicsSamples.end(), [](const KinematicsSample &k1, const KinematicsSample &k2)
                  { return k1.time < k2.time; });
    }

    // ----- PArameters

    LidarDiffDriveParameters::LidarDiffDriveParameters(const Eigen::Vector3d &initial_kinematics_parameters, int tics_per_revolution, const Eigen::Vector3d &initial_sensor_pose_parameters) : _ticsPerRevolution(tics_per_revolution)
    {
        _k = Eigen::VectorXd::Zero(6);
        _k[0] = initial_kinematics_parameters[0];
        _k[1] = initial_kinematics_parameters[1];
        _k[2] = initial_kinematics_parameters[2];

        _k[3] = initial_sensor_pose_parameters[0];
        _k[4] = initial_sensor_pose_parameters[1];
        _k[5] = initial_sensor_pose_parameters[2];
    }
    double LidarDiffDriveParameters::kl() const
    {
        return _k[0] * 2 * M_PI / _ticsPerRevolution;
    }
    double LidarDiffDriveParameters::kr() const
    {
        return _k[1] * 2 * M_PI / _ticsPerRevolution;
    }
    double LidarDiffDriveParameters::kb() const
    {
        return _k[2];
    }
    LidarDiffDriveParameters LidarDiffDriveParameters::addPerturbation(const Eigen::VectorXd &perturbation) const
    {
        Eigen::Vector3d kinematics_params = ManifoldCalibration::addPerturbation(kinematicsParameters(), kinematicsPerturbation(perturbation));
        Eigen::Isometry2d sensor_pose = fromVector(sensorParameters());
        Eigen::Vector3d sensor_params = toVector(ManifoldCalibration::addPerturbation(sensor_pose, sensorPerturbation(perturbation)));
        return LidarDiffDriveParameters(kinematics_params, _ticsPerRevolution, sensor_params);
    }
    Eigen::Vector3d LidarDiffDriveParameters::kinematicsParameters() const
    {
        return Eigen::Vector3d{_k[0], _k[1], _k[2]};
    }
    Eigen::Vector3d LidarDiffDriveParameters::sensorParameters() const
    {
        return Eigen::Vector3d{_k[3], _k[4], _k[5]};
    }

    Eigen::Vector3d LidarDiffDriveParameters::sensorPerturbation(const Eigen::VectorXd &perturbation) const
    {
        return Eigen::Vector3d(perturbation[3], perturbation[4], perturbation[5]);
    }
    Eigen::Vector3d LidarDiffDriveParameters::kinematicsPerturbation(const Eigen::VectorXd &perturbation) const
    {
        return Eigen::Vector3d(perturbation[0], perturbation[1], perturbation[2]);
    }
    void LidarDiffDriveParameters::print() const
    {
        std::cout << " K=" << _k << std::endl;
    }

    // ----- Solver2d

    Solver2d::Solver2d(const DifferentialDriveKinematics &differentialKinematics, const Eigen::Vector3d &initialKinematicsParameters, const Eigen::Vector3d &initialSensorPose, int iterations) : _differentialKinematics(differentialKinematics), _k(initialKinematicsParameters, differentialKinematics.ticsPerRevolution(), initialSensorPose), _iterations(iterations)
    {
    }

    void Solver2d::addKinematicsSample(const KinematicsSample &kinematics_sample)
    {
        _dataset.addSample(kinematics_sample);
    }

    void Solver2d::addSensorSample(const SensorSample &sensor_sample)
    {
        _dataset.addSample(sensor_sample);
    }

    void Solver2d::solve()
    {
        std::cout << " initial kinematics: :" << _k.kinematicsParameters() << std::endl;
        std::cout << " initial sensor_pose :" << _k.sensorParameters() << std::endl;
        _dataset.sort();
        // _dataset.debuginfo();

        for (size_t iter = 0; iter < _iterations; iter++)
        {
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, 6);
            // Information matrix of the laser scan matcher
            Eigen::MatrixXd Omega = Eigen::MatrixXd::Identity(3, 3);
            Eigen::VectorXd b = Eigen::VectorXd::Zero(6);
            double chi_squared = 0.;
            bool non_empty_batch = false;
            for (size_t i = 0; i < _dataset.size(); i++)
            {
                const DatasetSample &sample = _dataset.sample(i);
                if (sample.kinematicsSamples.empty())
                {
                    // std::cout << "kinematicsSamples empty: skipping" << std::endl;
                    continue;
                }

                if (sample.skip())
                {
                    // std::cout << "skipping sample since all encoder tics are zero" << std::endl;
                    continue;
                }
                non_empty_batch = true;
                Eigen::Isometry2d sensor_relative_increment = sample.sensorSample.relativeTransf;
                // std::cout << " ====== BEGIN ======= "
                //           << " sample " << i << std::endl;
                // std::cout << "sample.kinematicsSamples.size()=" << sample.kinematicsSamples.size() << " time in sensorSample: " << sample.sensorSample.time << std::endl;
                // for (int j = 0; j < sample.kinematicsSamples.size(); j++)
                //     std::cout << " time in kinematics sample: " << sample.kinematicsSamples[j].time << std::endl;
                Eigen::Isometry2d odometry_rel_increment = odometryRelativeIncrement(_k, sample);

                // std::cout << "sensor_relative_increment in sensor coordinates " << toVector(sensor_relative_increment) << std::endl;
                // std::cout << " accTicsL= " << sample.accTicsL() << " accTicsR=" << sample.accTicsR() << std::endl;

                // std::cout << " ====== END =======" << std::endl;
                //  prediction of the sensor frame of reference given its currently estimated local pose and the encoder tics
                Eigen::Isometry2d currentSensorPose = fromVector(_k.sensorParameters());
                Eigen::Isometry2d h_i = h(odometry_rel_increment, currentSensorPose);
                /// std::cout << " h_i (prediction)= " << toVector(h_i) << std::endl;
                // error (column vector)
                Eigen::VectorXd e_i = error(h_i, sensor_relative_increment);
                // std::cout << " e_i (error)= " << e_i << std::endl;

                Eigen::MatrixXd J_i = jacobian(sample, sensor_relative_increment);
                // update H and b
                //           6x3           3x3    3x6
                H = H + J_i.transpose() * Omega * J_i;
                //           6x3           3x3    3x1
                b = b + J_i.transpose() * Omega * e_i;

                // std::cout << " > b= " << b << std::endl;
                chi_squared += e_i.norm();
            }

            if (non_empty_batch)
            {
                // solve Hx = -b
                Eigen::VectorXd deltaX = H.ldlt().solve(-b);
                std::cout << " ======================================================== OPTIMIZATION ==========================================================" << std::endl;
                std::cout << " H=" << H << std::endl;
                std::cout << " b=" << b << std::endl;
                std::cout << " chi_squared=" << chi_squared << std::endl;
                std::cout << " iteration: " << iter << " deltaX :" << deltaX << std::endl;

                //  finally update the variable to optimize
                _k = _k.addPerturbation(deltaX);

                std::cout << " kinematics: :" << _k.kinematicsParameters() << std::endl;
                std::cout << " sensor_pose :" << _k.sensorParameters() << std::endl;
                std::cout << " ======================================================== OPTIMIZATION END ==========================================================" << std::endl;
            }
        }
    }

    Eigen::Isometry2d Solver2d::odometryRelativeIncrement(const LidarDiffDriveParameters &k, const DatasetSample &sample) const
    {
        Eigen::Isometry2d odometry_relative_increment = fromVector(Eigen::Vector3d(0., 0., 0.));

        for (const auto &kinematics_sample : sample.kinematicsSamples)
        {
            // std::cout << " => tics left right: " << kinematics_sample.deltaTicsLeft << " " << kinematics_sample.deltaTicsRight << std::endl;
            Eigen::Isometry2d relative_tf = _differentialKinematics.kinematics(k.kl(), k.kr(), k.kb(), kinematics_sample.deltaTicsLeft, kinematics_sample.deltaTicsRight);
            // std::cout << "relative_tf from tics: " << toVector(relative_tf) << std::endl;
            odometry_relative_increment = relative_tf * odometry_relative_increment;
            // std::cout << "odometry_relative_increment " << toVector(odometry_relative_increment) << " <=" << std::endl;
        }
        return odometry_relative_increment;
    }

    Eigen::Isometry2d Solver2d::h(
        const Eigen::Isometry2d &base_relative_tf, const Eigen::Isometry2d &sensor_pose) const
    {
        return sensor_pose.inverse() * base_relative_tf;
    }

    /*
     @prediction: the predicted relative motion of the sensor frame according to wheel odometry
     @measurement: the measured relative motion of the sensor frame according to scan matching

    */
    Eigen::Vector3d Solver2d::error(const Eigen::Isometry2d &prediction, const Eigen::Isometry2d &measurement) const
    {
        return toVector(measurement.inverse() * prediction);
    }

    Eigen::MatrixXd Solver2d::jacobian(const DatasetSample &sample, const Eigen::Isometry2d &sensor_relative_increment) const
    {
        constexpr size_t ksize = 6;
        //                     0.035
        constexpr double eps = 0.000001;
        // 3x6 : 3 coordinates in SE(2) x 6 parameters in k
        Eigen::MatrixXd jacobian_matrix = Eigen::MatrixXd::Zero(3, 6);
        // iterate over the k parameters: (Kl kr kb kx ky ktheta)
        for (size_t i = 0; i < ksize; i++)
        {
            Eigen::VectorXd perturbation = Eigen::VectorXd::Zero(ksize);
            // E_plus
            perturbation(i) = eps;
            LidarDiffDriveParameters k = _k.addPerturbation(perturbation);
            // std::cout << " jacobian perturbing parameter# " << i << std::endl;
            // k.print();
            Eigen::Vector3d pose_parameters = k.sensorParameters();
            Eigen::Isometry2d current_sensor_pose = fromVector(pose_parameters);

            Eigen::Isometry2d odometry_rel_increment = odometryRelativeIncrement(k, sample);
            Eigen::Isometry2d h_i = h(odometry_rel_increment, current_sensor_pose); // odometryRelIncrement * currentSensorPose;
            Eigen::Vector3d e_plus = error(h_i, sensor_relative_increment);

            // E_minus
            perturbation(i) = -eps;
            k = _k.addPerturbation(perturbation);
            pose_parameters = k.sensorParameters();
            current_sensor_pose = fromVector(pose_parameters);

            odometry_rel_increment = odometryRelativeIncrement(k, sample);
            h_i = h(odometry_rel_increment, current_sensor_pose); // odometryRelIncrement * currentSensorPose;
            Eigen::Vector3d e_minus = error(h_i, sensor_relative_increment);
            // std::cout << "i=" << i << "(x y theta)" << std::endl;
            // std::cout << " sensor_relative_increment = " << toVector(sensor_relative_increment) << std::endl;
            // std::cout << " e_plus = " << e_plus << std::endl;
            // std::cout << " e_minus = " << e_minus << std::endl;
            Eigen::Vector3d column = (e_plus - e_minus) * 0.5 / eps;
            // std::cout << " column = " << column << std::endl;
            jacobian_matrix.col(i) = column;
        }

        // std::cout << " jacobianMatrix = " << jacobian_matrix << std::endl;

        return jacobian_matrix;
    }

}