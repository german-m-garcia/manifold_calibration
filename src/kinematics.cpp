#include "kinematics.h"
#include "conversions.h"

#include <math.h>
#include <iostream>

namespace ManifoldCalibration
{
    DifferentialDriveKinematics::DifferentialDriveKinematics(int ticsPerRevolution) : _ticsPerRevolution(ticsPerRevolution)
    {
    }

    int DifferentialDriveKinematics::ticsPerRevolution() const
    {
        return _ticsPerRevolution;
    }

    Eigen::Isometry2d DifferentialDriveKinematics::kinematics(double kl, double kr, double kb, int ticsL, int ticsR) const
    {
        // std::cout << "kl kr kb=" << kl << " " << kr << " " << kb << std::endl;
        double x = (kr * ticsR + kl * ticsL) / 2.;
        double theta = (kr * ticsR - kl * ticsL) / (2. * kb);
        return fromVector(Eigen::Vector3d(x, 0., theta));
    }

    Eigen::Vector3d DifferentialDriveKinematics::kParameters(double rl, double rr, double b) const
    {
        double kl = kFromRadius(rl);
        double kr = kFromRadius(rr);

        return Eigen::Vector3d({kl, kr, b});
    }

    double DifferentialDriveKinematics::kFromRadius(double radius) const
    {
        return kFromRadius(radius, _ticsPerRevolution);
    }

    double DifferentialDriveKinematics::radiusFromK(double k) const
    {
        return radiusFromK(_ticsPerRevolution);
    }

    double DifferentialDriveKinematics::kFromRadius(double radius, int ticsPerRevolution) const
    {
        const double l = 2. * M_PI * radius;
        return l / (1. * ticsPerRevolution);
    }

    double DifferentialDriveKinematics::radiusFromK(double k, int ticsPerRevolution) const
    {
        return k * (1. * ticsPerRevolution) / (2. * M_PI);
    }

}