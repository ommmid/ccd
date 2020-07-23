#pragma once

#include <Eigen/Geometry>
#include <random>

#include <stdexcept>
#include <string>

#include "ccd/base/calculator.h"

namespace ccd
{

struct IKInput
{
    /** \brief Desired pose */ 
    Eigen::Affine3d& target;

    double threshold;
    
    double requested_iteration_num;
    
    CalculatorType calculator_type;
};


struct IKOutput
{
    /** \brief Joints values at the solution */ 
    std::vector<double> solution_joints_values;

    /** \brief Some sort of error */
    double target_ee_error;

    /** \brief To track the error between the ee and target
     * the first error is the initial one, between the ee at the initial configuration and the 
     * given target
     */
    std::vector<double> target_ee_error_track;

    /** \brief Number of iterations */
    int final_iteration_num;

    /** \brief a data structure to store all the frames in forward and backward reaching
     * for all the iterations.
     * each row (inner vector) is a "frames_" and the first row is backward reaching followed
     * by a forward reaching in the second row. Obviously the last row is a forward reaching.
     */
    std::vector<std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>> frames_matrix;

    // ----------- for debuggin purposes
    // the first row is for backward reaching and has one less than dof element
    // the elements include the vector that connect the start to aim in world frame
    // the second row is for forward reaching and the number of elements are 
    // equal to dof
    std::vector<std::vector<Eigen::Vector3d>> start_to_aim_track;
    
    std::vector<std::vector<Eigen::Vector3d>> start_to_aim_projected_track;
    std::vector<std::vector<Eigen::Vector3d>> start_to_end_track;
    std::vector<std::vector<Eigen::Vector3d>> start_to_end_projected_track;

    std::vector<std::vector<double>> angle_track;
};



}

