#pragma once

#include <Eigen/Geometry>
#include <random>

#include <stdexcept>
#include <string>

// copied from ompl
namespace ccd
{

/** \brief An affine3d (4x4) matrix with only rotation around z and no translation */
Eigen::Affine3d rotation_z(const double& theta);

/** \brief Random number between two numbers */
double randomDouble(const double start, const double end);

/** \brief Find the angle between two arbitrary vectors. It finds a positive angle that
 * goes FROM v1 TO v2
 * \param v1 is the frist vector that we want to measure the angle FROM
 * \param v2 is the second vector that we want to measure the angle TO
 * \param n is a unit vector showing direction we want to measure the angle around
 */
double signedAngleBetweenTwoVectors(Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& n);

std::vector<std::complex<double>> find4Root(double a, double b, double c, double d, double e);

}

