/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "ccd/robot_state/robot_state.h"
#include "ccd/util/exception.h"
#include "ccd/util/math.h"
#include "ccd/util/io.h"

#include "ccd/base/calculator.h"


namespace ccd
{

PositionBasedCalculator::PositionBasedCalculator()
{
}

double PositionBasedCalculator::calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2)
{
    // distance between the location of two frames is the error
    Eigen::Vector3d frame_1_to_frame_2 = frame_2.translation() - frame_1.translation();
    return frame_1_to_frame_2.norm();
}


std::vector<double> PositionBasedCalculator::calculateReach(const Eigen::Vector4d oe, 
                                                            const Eigen::Vector4d xe,
                                                            const Eigen::Vector4d ze,
                                                            const Eigen::Vector4d ote, 
                                                            const Eigen::Vector4d xte,
                                                            const Eigen::Vector4d zte)
{
    double A = oe[0] * oe[1]  + xe[0] * xe[1]  + ze[0] * ze[1];
    double B = oe[1] * ote[0] - oe[0] * ote[1] + xe[1] * xte[0] - xe[0] * xte[1] + ze[1] * zte[0] - ze[0] * zte[1];
    double D = oe[0] * ote[0] + oe[1] * ote[1] + xe[0] * xte[0] + xe[1] * xte[1] + ze[0] * zte[0] + ze[1] * zte[1];
    double E = oe[2] * oe[2] - oe[0] * oe[1] - ote[2] * oe[2] + 
               xe[2] * xe[2] - xe[0] * xe[1] - xte[2] * xe[2] + 
               ze[2] * ze[2] - ze[0] * ze[1] - zte[2] * ze[2];

    double H = E - A;

    // coeffecients of the 4th degree polynomial 
    double C4 = 4 * A * A;
    double C3 = 4 * A * B;
    double C2 = B * B + 4 * A * H + D * D;
    double C1 = 2 * B * H;
    double C0 = H * H - D * D;

    const double IMAGINARY_ERROR = 1e-5;
    std::vector<std::complex<double>> solutions = ccd::find4Root(C4, C3, C2, C1, C0);
    std::vector<double> real_solutions;

    // extract the real solutions
    for (auto solution : solutions)
    {
        if(abs(std::imag(solution)) < IMAGINARY_ERROR)
        {
            double angle_solution = std::acos(std::real(solution));
            real_solutions.push_back(angle_solution);
            // arccosine can have two solution
            real_solutions.push_back(-angle_solution);
        }
    }

    return real_solutions; 
}

OrientationBasedCalculator::OrientationBasedCalculator()
{

}

double OrientationBasedCalculator::calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2)
{
    
    return 0;
}

std::vector<double> OrientationBasedCalculator::calculateReach(const Eigen::Vector4d oe, 
                            const Eigen::Vector4d xe,
                            const Eigen::Vector4d ze,
                            const Eigen::Vector4d ote, 
                            const Eigen::Vector4d xte,
                            const Eigen::Vector4d zte)
{
    
    // what is the joint angle after calculating the angle between two vectors ???
    std::vector<double> v = {10};
    return  v; // correct it
}


ComboCalculator::ComboCalculator()
{

}


double ComboCalculator::calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2)
{
    
    return 0;
}

std::vector<double> ComboCalculator::calculateReach(const Eigen::Vector4d oe, 
                            const Eigen::Vector4d xe,
                            const Eigen::Vector4d ze,
                            const Eigen::Vector4d ote, 
                            const Eigen::Vector4d xte,
                            const Eigen::Vector4d zte)
{
    
    // what is the joint angle after calculating the angle between two vectors ???
    std::vector<double> v = {10};
    return  v; // correct it
}


}