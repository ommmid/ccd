#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Fabrik"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/world/world.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"
#include "fabrik/robot_model/robot_model.h"
#include "fabrik/robot_state/robot_state.h"

#include "fabrik/util/output.h"

#include "fabrik/base/calculator.h"
#include "fabrik/base/fabrik.h"


const double TRANSLATION_ERROR = 1e-6;
const double ANGLE_ERROR = 1e-5;
const double ROTATION_ERROR = 1e-5;

BOOST_AUTO_TEST_CASE(FABRIK2D)
{
    fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot2D();
    fabrik::RobotStatePtr robot_state_1 = 
        std::make_shared<fabrik::RobotState>(robot_model);

    // ---------------------- Solve a forward kinematics first:
    robot_state_1->setReachingDirection(fabrik::ReachingDirection::FORWARD);
    std::vector<double> fk_joints_values_1 = {M_PI_4, 0, 0};
    for (int k = 0; k < 3; ++k)
        robot_state_1->updateState(fk_joints_values_1[k], k);

    robot_state_1->printState("FABRIK - first configuration", std::vector<int>{0,1,2});
    
    // The length of the links from makeSimpleRobot2D is 1 and in local x direction
    Eigen::Affine3d end_effector_1 = robot_state_1->getFrames(2).second;
    Eigen::Vector3d expected_location_1(3 * std::cos(M_PI_4),
                                        3 * std::sin(M_PI_4),
                                        0);

    for(int k = 0; k < 3; ++k)
    {
        BOOST_TEST(abs(end_effector_1.translation()[k] - expected_location_1(k)) < TRANSLATION_ERROR);
    }

    // ---------------------- Solve another forward kinematics close to the first one:
    fabrik::RobotStatePtr robot_state_2 = 
        std::make_shared<fabrik::RobotState>(robot_model);
    
    robot_state_2->setReachingDirection(fabrik::ReachingDirection::FORWARD);
    double theta_1 = M_PI_4 + 0.1;
    double theta_2 = 0.1;
    double theta_3 = 0.1;
    std::vector<double> fk_joints_values_2 = {theta_1, theta_2, theta_3};
    for (int k = 0; k < 3; ++k)
        robot_state_2->updateState(fk_joints_values_2[k], k);

    robot_state_2->printState("FABRIK - second configuration", std::vector<int>{0,1,2});

    Eigen::Affine3d end_effector_2 = robot_state_2->getFrames(2).second;
    Eigen::Vector3d expected_location_2(
        std::cos(theta_1) + std::cos(theta_1 + theta_2) + std::cos(theta_1 + theta_2 + theta_3),
        std::sin(theta_1) + std::sin(theta_1 + theta_2) + std::sin(theta_1 + theta_2 + theta_3),
                                      0);

    for(int k = 0; k < 3; ++k)
    {
        BOOST_TEST(abs(end_effector_2.translation()[k] - expected_location_2(k)) < TRANSLATION_ERROR);
    }

    // get the end effector frame of the second state
    Eigen::Affine3d target = end_effector_2;
    double threshold = 0.01;
    double requested_iteration_num = 100;

    fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(robot_model, fk_joints_values_1));

    fabrik->setInverseKinematicsInput(target,
                                     threshold,
                                     requested_iteration_num,
                                     fabrik::CalculatorType::POSITION);
    
    fabrik::IKOutput output;
    bool solved = fabrik->solveIK(output);

    if(solved)
    {
        std::cout << "solveIK was successful" << std::endl;
        std::cout << "total iteration: " << output.final_iteration_num << std::endl;
        std::cout << "error: " << output.target_ee_error << std::endl;
    }
}

BOOST_AUTO_TEST_CASE(FABRIK3D)
{
   

}

