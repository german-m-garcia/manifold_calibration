#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Geometry>

namespace ManifoldCalibration
{
    class DifferentialDriveKinematics
    {
    public:
        DifferentialDriveKinematics(int ticsPerRevolution);
        int ticsPerRevolution() const;
        Eigen::Isometry2d kinematics(double kl, double kr, double kb, int ticsL, int ticsR) const;

        Eigen::Vector3d kParameters(double rl, double rr, double b) const;
        double kFromRadius(double radius) const;
        double radiusFromK(double k) const;

    private:
        double kFromRadius(double radius, int ticsPerRevolution) const;
        double radiusFromK(double k, int ticsPerRevolution) const;

        int _ticsPerRevolution;
    };
}

#endif