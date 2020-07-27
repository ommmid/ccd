/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "ccd/robot_model/robot_model.h"
#include "ccd/util/class_forward.h"
#include "ccd/base/calculator.h"
#include "ccd/util/io.h"

#include "ccd/robot_state/robot_state.h"

namespace ccd
{  

CCD_CLASS_FORWARD(CCD);

/** \brief This class includes the problem input, output and the solver
 */
class CCD
{
public:
    /** \brief Construct a FABRIK object with a robot_model
     * robot_state will be set to all-joint-zero position
     */
    CCD( const RobotModelPtr& robot_model);

    /** \brief Construct a FABRIK object with a robot_model
     * robot_state will be set to initial configuration
     */
    CCD( const RobotModelPtr& robot_model, const std::vector<double>& initial_configuration);
    
    void setInverseKinematicsInput(const std::vector<Eigen::Vector4d>& target_3points,
                                   const double threshold,
                                   const double requested_iteration_num,
                                   const CalculatorType calculator_type);

    /** \brief Solve inverse kinematics */
    bool solveIK(IKOutput& output);

    /** \brief Solve forward kinematics */
    bool solveFK(const std::vector<double>& configuration, Eigen::Affine3d& target);

    // TODO[Omid]: how to get a const robot state to return from fabrik object ?????
    // fabrik::RobotStateConstPtr getConstRobotState()
    // {
    //     fabrik::RobotStateConstPtr const_robot_state = robot_state_;
    //     return 
    // }

private:

ccd::RobotStatePtr robot_state_;

// int dof_;

/** \brief Joint values at the initial configuration. */
std::vector<double> initial_configuration_;

/** \brief */
Eigen::Affine3d target_;

/** \brief The target we want to reach to. Three points on a rigid body. */
std::vector<Eigen::Vector4d> target_3points_;

double threshold_;
int requested_iteration_num_;

CalculatorPtr calculator_;

/** \brief calculate the objective function at the current state */
double objectiveFunction();

/** \brief calculate the objective function at a given configuration. We need this becasues we might want to check
 * the objective function at some configuration without chaing the main robot_state_. That is why we make another
 * robot state in this function to check the objective function
 */
double objectiveFunction(const std::vector<double>& angles, const std::vector<int>& joints_numbers);

CalculatorPtr createCalculator(const CalculatorType& calculator_type);

// Here the scenarios for reaching could be different:
// BackwardReaching: all the joints are updated from ee to 0
// ForwardReaching: all the joints are updated from 0 to ee
// SmartReaching: some criteria to choose which joints in what order should be updated

void backwardReaching(IKOutput& output);

void forwardReaching(IKOutput& output);

void smartReaching(IKOutput& output);

};



}