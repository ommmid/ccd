/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "ccd/robot_model/robot_model.h"
#include "ccd/util/class_forward.h"

namespace ccd
{   
// I want forward decleration for RobotState becuase I predict it will be a heavy class
CCD_CLASS_FORWARD(RobotState);

/** \brief This class determines that what state our chain is at. Considering FABRIK
 * algorithm, the state of the robot are the things that we want to update.
 * Joint number and link number are the same. And the numbering starts from
 * 0 so we do not have difficulty in indexing
 */
class RobotState
{
public:
    /** \brief Construct a state at a forward-completed home configuratn. It means:
     * reached_at_ = dof - 1
     * reachin_direction_ = FORWARD
     * joint_values = set to zeros
     */
    RobotState(const RobotModelPtr& robot_model);

    /** \brief Construct a state at a forward-completed by a given configuration. It means:
     * reached_at_ = dof - 1
     * reachin_direction_ = FORWARD
     * joint_values = set to the given configuration
     */
    RobotState(const RobotModelPtr& robot_model, const std::vector<double>& given_configuration);

    // /** \brief Not sure if I need to create any other constructor
    //  */
    // RobotState(const std::vector<fabrik::Link>& chain,
    //            const Eigen::Affine3d& base,
    //            const ReachingDirection& reaching_direction);


    /** \brief This function updates only [s_i e_i] in frames_ and those after that in the chain, 
     * frames start_{i+1} and end_{i+1} ... because this function is used in
     * either forward or backward reaching, meaning it does not
     * calculate a full forward kinematics to reach to the end-effector. It is being disassembled
     * Joint number and link number are the same. J1 is on s1. s1 and e1 are both on link 1
     * and they start from 0 to n-1, n being the number of degrees of freedom
     * It also updates the joints values first
     */ 
    void updateState(const std::vector<double>& joints_values, const std::vector<int>& joints_numbers);

    const std::vector<double> getJointsValues() const
    {
        return joints_values_;
    }

    const double getJointsValues(int index) const
    {
        return joints_values_[index];
    }

    const std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> getFrames() const
    {
        return frames_;
    } 

    std::pair<Eigen::Affine3d, Eigen::Affine3d> getFrames(int link_index) const
    {
        return frames_[link_index];
    } 

    const RobotModelPtr getRobotModel() const
    {
        return robot_model_;
    }

    /** \brief get the point O in the world frame */
    const Eigen::Vector4d getPoint1() const
    {
        return frames_[dof_ - 1].second * robot_model_->getEEPointO();
    }

    const Eigen::Vector4d getPoint2() const
    {
        return frames_[dof_ - 1].second * robot_model_->getEEPointX();
    }

    const Eigen::Vector4d getPoint3() const
    {
        return frames_[dof_ - 1].second * robot_model_->getEEPointZ();
    }

    /** \brief the x point on the end_frame of the last link expressed requested start_frame */
    const Eigen::Vector4d getPoint1InLinkStartFrame(int link_number) const;

    const Eigen::Vector4d getPoint2InLinkStartFrame(int link_number) const;

    const Eigen::Vector4d getPoint3InLinkStartFrame(int link_number) const;
 
    /** \brief for debugging purposes */
    std::string createDashBoard() const;

    /** \brief print out the information of the current state 
     * \param text a text showing where this function is called from and more info if needed
     * \param which_frames which frames to show the transformation for
    */
    void printState(const std::string text, const std::vector<int>& which_frames) const;

private:
 
    /** \brief Degrees of freedom  */
    int dof_;

    /** \brief The value of each joint for the current state. Joint number start from 0. */
    std::vector<double> joints_values_;

    /** \brief Chain. The shared pointer is pointing to a const fabrik::Chain
     * because we wont change this chain (it is the structure of the robot)
     */
    RobotModelPtr robot_model_;

    std::vector<Link> chain_;

    /** \brief All frames of all links in order, expressed in world frame:
     * [s_0 e_0 s_1 e_1 .... s_n-1 e_n-1]. Because I want to keep the indecis easily 
     * to read. I will make pair for each start and end:
     * [[s_0 e_0] [s_1 e_1] .... [s_n-1 e_n-1]]
     */
    std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>> frames_;

    /** \brief Base frame. {e_b}, the end frame of the base link, expressed
     *  in the world coordinate: {e_b}_w.
     *  to show the frame, the convention will be:
     *  <start/end>_<frame number>_<the frame this frame is expressed in>
     */
    Eigen::Affine3d base_;

};



}