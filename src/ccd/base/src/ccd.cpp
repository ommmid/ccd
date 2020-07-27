/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "ccd/robot_model/robot_model.h"
#include "ccd/robot_state/robot_state.h"
#include "ccd/util/exception.h"
#include "ccd/util/math.h"
#include "ccd/util/io.h"

#include "ccd/base/ccd.h"
#include "ccd/base/calculator.h"

namespace ccd
{

CCD::CCD( const RobotModelPtr& robot_model)
{
    // We have a robot state internally
    robot_state_ = std::make_shared<ccd::RobotState>(robot_model); 
}

CCD::CCD( const RobotModelPtr& robot_model, const std::vector<double>& initial_configuration):
initial_configuration_(initial_configuration)
{
    // set the state of the robot to the given initial_configuration
    robot_state_ = std::make_shared<ccd::RobotState>(robot_model, initial_configuration_);
}

void CCD::setInverseKinematicsInput(const std::vector<Eigen::Vector4d>& target_3points,
                const double threshold,
                const double requested_iteration_num,
                const CalculatorType calculator_type)
{
    target_3points_ = target_3points;
    threshold_ = threshold;
    requested_iteration_num_ = requested_iteration_num;

    // set the calculator 
    calculator_ = CCD::createCalculator(calculator_type);
}

bool CCD::solveFK(const std::vector<double>& configuration, Eigen::Affine3d& target)
{
    // I have to make a new object with a new pointer. Getting a copy of "robot_state_" will make
    // another shared pointer that is pointing to the same robot_state object and I do not want that.
    // However, I can get some const information like robot_model and dof from any robot_state
    ccd::RobotStatePtr robot_state_fk = 
            std::make_shared<ccd::RobotState>(robot_state_->getRobotModel(), configuration);

    int dof = robot_state_fk->getRobotModel()->getDOF();
    target = robot_state_fk->getFrames(dof - 1).second;

    return true;
}


CalculatorPtr CCD::createCalculator(const CalculatorType& calculator_type)
{
    switch (calculator_type)
    {
        case CalculatorType::POSITION:
            return CalculatorPtr(new PositionBasedCalculator());
            break;
        case CalculatorType::ORIENTATION:
            return CalculatorPtr(new OrientationBasedCalculator());
            break;
        case CalculatorType::COMBINATION:
            return CalculatorPtr(new ComboCalculator());
            break;
    }
}

bool CCD::solveIK(IKOutput& output)
{
std::cout << "========================== CCD sooooolllvvvvveeeeee IK ======================" << std::endl;

int dof = robot_state_->getRobotModel()->getDOF();

// end_effector: the end frame of the last link
Eigen::Affine3d end_effector = robot_state_->getFrames(dof - 1).second;
double target_ee_error = objectiveFunction(); 
output.target_ee_error_track.push_back(target_ee_error);

std::cout << "initial error: \n" << target_ee_error << std::endl;
std::cout << "target: \n" << target_.matrix() << std::endl;
robot_state_->printState("ccd::ccd.cpp: Initial Configuration", std::vector<int>{0,1,2});

int iteration_num = 0;
while (target_ee_error > threshold_ && (iteration_num != requested_iteration_num_))
{
    std::cout << "------------------------ iteration number: " << iteration_num << std::endl;
    
    // robot_state_->printState("ccd::ccd.cpp: backwardReaching", std::vector<int>{0,1,2});
    
    // do one backward reaching
    CCD::backwardReaching(output);
    output.frames_matrix.push_back(robot_state_->getFrames());

    end_effector = robot_state_->getFrames(dof - 1).second;
    target_ee_error = objectiveFunction(); 
    output.target_ee_error_track.push_back(target_ee_error);

    std::cout << "\ntarget_ee_error: " << target_ee_error << std::endl;
    robot_state_->printState("ccd::ccd.cpp: backwardReaching", std::vector<int>{0,1,2});

    ++iteration_num;
}

output.final_iteration_num = iteration_num;
output.solution_joints_values = robot_state_->getJointsValues();

output.target_ee_error = target_ee_error;

return true;
}

double CCD::objectiveFunction()
{
    int dof = robot_state_->getRobotModel()->getDOF();
    std::vector<Eigen::Vector4d> ee_3points = robot_state_->getRobotModel()->getEE3Points();

    Eigen::Vector4d ee_o = robot_state_->getFrames(dof - 1).second * ee_3points[0];
    Eigen::Vector4d ee_x = robot_state_->getFrames(dof - 1).second * ee_3points[1];
    Eigen::Vector4d ee_z = robot_state_->getFrames(dof - 1).second * ee_3points[2];

    return  (ee_o - target_3points_[0]).dot(ee_o - target_3points_[0]) + 
            (ee_x - target_3points_[1]).dot(ee_x - target_3points_[1]) + 
            (ee_z - target_3points_[2]).dot(ee_z - target_3points_[2]);
}

double CCD::objectiveFunction(const std::vector<double>& angles, const std::vector<int>& joints_numbers)
{
    int dof = robot_state_->getRobotModel()->getDOF();
    std::vector<Eigen::Vector4d> ee_3points = robot_state_->getRobotModel()->getEE3Points();

    // objective function has a separate state because I do not want to mess up the main robot_state_
    RobotStatePtr robot_state_local = std::make_shared<ccd::RobotState>(robot_state_->getRobotModel(), robot_state_->getJointsValues());
    robot_state_local->updateState(angles, joints_numbers);
    Eigen::Vector4d ee_o = robot_state_local->getFrames(dof - 1).second * ee_3points[0];
    Eigen::Vector4d ee_x = robot_state_local->getFrames(dof - 1).second * ee_3points[1];
    Eigen::Vector4d ee_z = robot_state_local->getFrames(dof - 1).second * ee_3points[2];

    return  (ee_o - target_3points_[0]).dot(ee_o - target_3points_[0]) + 
            (ee_x - target_3points_[1]).dot(ee_x - target_3points_[1]) + 
            (ee_z - target_3points_[2]).dot(ee_z - target_3points_[2]);
}

void CCD::backwardReaching(IKOutput& output)
{
    int dof = robot_state_->getRobotModel()->getDOF();
       
    std::vector<double> angle_vec(dof);

    for(int joint_number = dof - 1; joint_number > -1; --joint_number)
    {
        // Example: For a 3DOF serial manipulator. To claculate the last joint (J_2), we need:
        // three-point on the target: target_3points_
        // three-point on the last link: getRobotModel()->getEE3Points(): ee_3points
        // oe: vector o from ee_3points relative to link_i frame
        // xe: vector x from ee_3points relative to link_i frame
        // ze: vector z from ee_3points relative to link_i frame
        // ote: vector ot from target_3points relative to the previous frame
        // xte: vector xt from target_3points relative to the previous frame
        // zte: vector zt from target_3points relative to the previous frame

        Eigen::Vector4d oe =  robot_state_->getPoint1InLinkStartFrame(joint_number);
        Eigen::Vector4d xe =  robot_state_->getPoint2InLinkStartFrame(joint_number);
        Eigen::Vector4d ze =  robot_state_->getPoint3InLinkStartFrame(joint_number);

        Eigen::Vector4d ote =  target_3points_[0].transpose() * robot_state_->getFrames(joint_number - 1).second.matrix();
        Eigen::Vector4d xte =  target_3points_[1].transpose() * robot_state_->getFrames(joint_number - 1).second.matrix();
        Eigen::Vector4d zte =  target_3points_[2].transpose() * robot_state_->getFrames(joint_number - 1).second.matrix();
        
        // the answer will be a set of real numbers representing cos_theta
        std::vector<double> reaching_angles = calculator_->calculateReach(oe, xe, ze, ote, xte, zte);
        
        // find angle that creates the min for objective function 
        double min_angle = reaching_angles[0];
        std::vector<double> min_angle_vec = {min_angle};
        std::vector<int> joint_number_vec = {joint_number};
        double min_objective_function = objectiveFunction(min_angle_vec, joint_number_vec);
        for (auto angle : reaching_angles)
        {
            std::vector<double> angle_vec = {angle};
            double objective_function = objectiveFunction(angle_vec, joint_number_vec);
            if (objective_function < min_objective_function)
            {
                min_objective_function = objective_function;
                min_angle = angle;
            }
        }
        
        // PositionBasedCalculatorPtr pbc = std::static_pointer_cast<PositionBasedCalculator>(calculator_);
        // start_to_aim_vec[joint_number] = pbc->start_to_aim;
        // start_to_aim_projected_vec[joint_number] = pbc->start_to_aim_projected;
        // start_to_end_vec[joint_number] = pbc->start_to_end;
        // start_to_end_projected_vec[joint_number] = pbc->start_to_end_projected;
        angle_vec[joint_number] = min_angle;

        min_angle_vec.clear(); min_angle_vec.push_back(min_angle);
        robot_state_->updateState(min_angle_vec, joint_number_vec);

        // robot_state_->printState("Forward Reaching .....", std::vector<int>{0,1,2});
    }

    for(int s = 0; s < angle_vec.size(); ++s)
        std::cout << "angle: " << angle_vec[s] << std::endl;
}


}