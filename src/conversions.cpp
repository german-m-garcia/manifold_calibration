#include "conversions.h"

namespace ManifoldCalibration
{
    Eigen::Vector3d toVector(const Eigen::Isometry2d &transform)
    {

        double theta = Eigen::Rotation2Dd(transform.linear()).angle();
        Eigen::Vector3d result(transform.translation().x(), transform.translation().y(), theta);
        return result;
    }

    Eigen::Isometry2d fromVector(const Eigen::Vector3d &vector)
    {
        Eigen::Isometry2d result;
        Eigen::Rotation2Dd rot(vector(2));
        Eigen::Matrix<double, 2, 2> rot_e(rot);
        Eigen::Matrix<double, 2, 1> trans(vector(0), vector(1));

        result.linear() = rot_e;
        result.translation() = trans;

        return result;
    }

    Eigen::Isometry2d addPerturbation(const Eigen::Isometry2d &transformation, const Eigen::Vector3d &perturbation)
    {
        return fromVector(perturbation) * transformation;
    }

    Eigen::VectorXd addPerturbation(const Eigen::VectorXd &transformation, const Eigen::VectorXd &perturbation)
    {
        return transformation + perturbation;
    }

}
