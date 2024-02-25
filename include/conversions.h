#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Geometry>

namespace ManifoldCalibration
{
    //! 2D Rotation Matrix -> (x, y, theta)
    Eigen::Vector3d toVector(const Eigen::Isometry2d &transform);

    //! (x, y, theta) -> 2D Rotation Matrix
    Eigen::Isometry2d fromVector(const Eigen::Vector3d &vector);

    Eigen::Isometry2d addPerturbation(const Eigen::Isometry2d &, const Eigen::Vector3d &);
    Eigen::VectorXd addPerturbation(const Eigen::VectorXd &transformation, const Eigen::VectorXd &perturbation);

}

#endif