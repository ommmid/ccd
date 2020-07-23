#include <Eigen/Geometry>
#include <random>

#include <stdexcept>
#include <string>
#include <complex>
#include <vector>

#include <ccd/util/math.h>


namespace ccd{

Eigen::Affine3d rotation_z(const double& theta)
{
    Eigen::Affine3d hm_z = Eigen::Affine3d(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    return hm_z;
} 

double randomDouble(const double start, const double end)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(start, end);

    return distribution(generator);
}

double signedAngleBetweenTwoVectors(Eigen::Vector3d& v1, Eigen::Vector3d& v2, Eigen::Vector3d& n)
{
    v1.normalize();
    v2.normalize();

    double cost = v1.dot(v2);
    double sint = (n.cross(v1)).dot(v2);
    double angle = std::atan2(sint, cost);

    // I could make angle to be positive between 0 and 2*pi but having it negative is OK too.

    return angle;
}

std::vector<std::complex<double>> find4Root(double a, double b, double c, double d, double e)
{
    std::complex<double> p1 = 2.0*pow(c,3) - 9.0*b*c*d + 27.0*a*pow(d,2) + 27.0*pow(b,2)*e - 72.0*a*c*e;
    std::complex<double> p2 = p1 + sqrt(-4.0 * pow(pow(c,2.0) - 3.0*b*d + 12.0*a*e, 3.0) + pow(p1,2.0));
    std::complex<double> p3 = (pow(c,2.0) - 3.0*b*d + 12.0*a*e)/(3.0*a* pow(p2/2.0, 1.0/3.0)) + pow(p2/2.0, 1.0/3.0)/(3.0*a);
    std::complex<double> p4 = sqrt( pow(b, 2.0)/(4 * pow(a, 2.0)) - (2*c)/(3*a)  + p3);
    std::complex<double> p5 = pow(b,2.0)/(2*pow(a,2.0)) - (4*c)/(3*a) - p3;
    std::complex<double> p6 = (-(pow(b,3.0)/pow(a,3.0))+(4*b*c/pow(a,2))-(8*d/a))/(4.0*p4);

    std::complex<double> x1 = -b/(4.0*a) - p4/2.0 - sqrt(p5 - p6)/2.0;
    std::complex<double> x2 = -b/(4.0*a) - p4/2.0 + sqrt(p5 - p6)/2.0;
    std::complex<double> x3 = -b/(4.0*a) + p4/2.0 - sqrt(p5 + p6)/2.0;
    std::complex<double> x4 = -b/(4.0*a) + p4/2.0 + sqrt(p5 + p6)/2.0;

    std::vector<std::complex<double>> v = {x1, x2, x3, x4};
    return v;
}

}




