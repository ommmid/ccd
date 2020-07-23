/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "ccd/robot_state/robot_state.h"
#include "ccd/util/exception.h"
#include "ccd/util/math.h"

namespace ccd
{

// -------------------- RobotState --------------------
RobotState::RobotState(const RobotModelPtr& robot_model):
robot_model_(robot_model)
{
    base_ = robot_model_->getBase();
    chain_ = robot_model_->getRobotChain();

    dof_ = robot_model_->getDOF();
    
    joints_values_.resize(dof_);
    frames_.resize(dof_);

    // Set all the joint to zero 
    for(int i = 0; i < dof_; ++i)
    {
        joints_values_[i] = 0;
    }
 
    // update the frames
    // The following will update all the frames starting from 0 all the way to ee
    updateState(joints_values_[0], 0);
}

RobotState::RobotState(const RobotModelPtr& robot_model, const std::vector<double>& given_configuration):
robot_model_(robot_model)
{
    base_ = robot_model_->getBase();
    chain_ = robot_model_->getRobotChain();

    dof_ = robot_model_->getDOF();
    
    joints_values_.resize(dof_);
    frames_.resize(dof_);

    // Set the joint values to the given ones and update the frames.
    for(int i = 0; i < dof_; ++i)
    {
        joints_values_[i] = given_configuration[i];
    }

    // frames number start from 0 
    // The following will update all the frames starting from 0 all the way to ee
    updateState(joints_values_[0], 0);
}


void RobotState::updateState(const double& joint_value, const int& joint_number)
{
    // std::cout << "joint number: " << joint_number << " joint value: " << joint_value << std::endl;

    joints_values_[joint_number] = joint_value;

    // update s_i and e_i and all the frames after that
    for (int j = joint_number; j < dof_; ++j)
    {
        // For index 0, we need the end frame of the base link
        Eigen::Affine3d end_i_minus_1 = (j == 0) ? base_ : frames_[j - 1].second;
        // s_i = e_{i-1} * rotation_z
        frames_[j].first = end_i_minus_1 * ccd::rotation_z(joints_values_[j]);
        // e_i = s_i * relative transformation of link_i
        frames_[j].second = frames_[j].first * chain_[j].getLinkFrame();
    }
    
    // std::cout << "rotation_z: \n" << fabrik::rotation_z(joint_value).matrix() << std::endl;
    // std::cout << "base: \n" << base_.matrix() << std::endl;
    // std::cout << "end_i_minus_1: \n" << end_i_minus_1.matrix() << std::endl;
    // std::cout << "start_i: \n" << frames_[joint_number].first.matrix() << std::endl;
    // std::cout << "end_i: \n" << frames_[joint_number].second.matrix() << std::endl;


    // printState("update state ......", std::vector<int>{0,1,2});
}

const Eigen::Vector4d RobotState::getOpointInLinkStartFrame(int link_number) const
{
    // x point on the ee expressed locally in the end_frame of the las link:
    Eigen::Vector4d o_ee = robot_model_->getEEPointO();
    Eigen::Affine3d start_frame_at_link_number = frames_[link_number].first;
    Eigen::Affine3d ee = frames_[dof_ - 1].second;

    return start_frame_at_link_number.inverse() * ee * o_ee;
}

const Eigen::Vector4d RobotState::getXpointInLinkStartFrame(int link_number) const
{
    // x point on the ee expressed locally in the end_frame of the las link:
    Eigen::Vector4d x_ee = robot_model_->getEEPointX();
    Eigen::Affine3d start_frame_at_link_number = frames_[link_number].first;
    Eigen::Affine3d ee = frames_[dof_ - 1].second;

    return start_frame_at_link_number.inverse() * ee * x_ee;
}

const Eigen::Vector4d RobotState::getZpointInLinkStartFrame(int link_number) const
{
    // x point on the ee expressed locally in the end_frame of the las link:
    Eigen::Vector4d z_ee = robot_model_->getEEPointZ();
    Eigen::Affine3d start_frame_at_link_number = frames_[link_number].first;
    Eigen::Affine3d ee = frames_[dof_ - 1].second;

    return start_frame_at_link_number.inverse() * ee * z_ee;
}


void RobotState::printState(const std::string text, const std::vector<int>& which_frames) const
{
    std::cout << "--------------------------- Robot State --------------------------- " << std::endl; 
    std::cout << "From: " << text << std::endl;

    std::string dash_board = createDashBoard();
    std::cout << dash_board << std::endl;

    Eigen::Affine3d end_effector = frames_[dof_ - 1].second;
    std::cout << "\nend effector: \n" << end_effector.matrix() << std::endl;

    if(which_frames[0] != -1)
    {
        std::cout << "base:\n" << base_.matrix() <<std::endl;
        for(int k : which_frames)
        {
            Eigen::MatrixXd start_HM = frames_[k].first.matrix();
            Eigen::MatrixXd end_HM = frames_[k].second.matrix();
            std::cout << "s" << std::to_string(k) << ": \n" << start_HM << std::endl;
            std::cout << "e" << std::to_string(k) << ": \n" << end_HM << std::endl;
        }
    }

    // Eigen::Affine3d ff = fabrik::rotation_z(0.2);
    // std::cout << ff.matrix() << std::endl;
    std::cout << "------------------------------------------------------------------- " << std::endl; 
}


std::string RobotState::createDashBoard() const
{
    std::string arrow = "<===";

    std::string dash_board = "      |";
    for (int k = 0; k < dof_; ++k)
    {
        std::string end_char = k == dof_-1 ? " " : "|";
        dash_board += "         " + end_char;
    }

    // second line
    dash_board += "\n[base]|";
    for (int k = 0; k < dof_; ++k)
    {
        if (k == dof_ - 1)
        {
            dash_board += "[s" + std::to_string(k) + "   e" + std::to_string(k) + "]";
        }else
        {
            dash_board += "[s" + std::to_string(k) + "   e" + std::to_string(k) + "]|";
        }
    }

    // third line
    dash_board += "\n      |";
    for (int k = 0; k < dof_; ++k)
    {
        std::string end_char = k == dof_-1 ? " " : "|";
        dash_board += "         " + end_char;        
    }

    // fourth line
    dash_board += "\n    ";
    for (int k = 0; k < dof_; ++k)
    {
        // to_string always creates 6 decmial digits. 
        dash_board += std::to_string(joints_values_[k]) + "  ";        
    }


return dash_board;
}


}