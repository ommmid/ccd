#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Ccd"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "ccd/util/exception.h"
#include "ccd/util/math.h"
#include "ccd/robot_model/robot_model.h"
#include "ccd/robot_state/robot_state.h"

#include "ccd/util/io.h"

#include "ccd/base/calculator.h"
#include "ccd/base/ccd.h"


const double TRANSLATION_ERROR = 1e-6;
const double ANGLE_ERROR = 1e-5;
const double ROTATION_ERROR = 1e-5;

inline double objectiveFunction(ccd::RobotStatePtr& robot_state, std::vector<Eigen::Vector4d>& target_3points)
{
    int dof = robot_state->getRobotModel()->getDOF();
    std::vector<Eigen::Vector4d> ee_3points = robot_state->getRobotModel()->getEE3Points();

    Eigen::Vector4d ee_1 = robot_state->getFrames(dof - 1).second * ee_3points[0];
    Eigen::Vector4d ee_2 = robot_state->getFrames(dof - 1).second * ee_3points[1];
    Eigen::Vector4d ee_3 = robot_state->getFrames(dof - 1).second * ee_3points[2];

    return  (ee_1 - target_3points[0]).dot(ee_1 - target_3points[0]) + 
            (ee_2 - target_3points[1]).dot(ee_2 - target_3points[1]) + 
            (ee_3 - target_3points[2]).dot(ee_3 - target_3points[2]);
}


BOOST_AUTO_TEST_CASE(CCD2D)
{
    ccd::RobotModelPtr robot_model = ccd::makeSimpleRobot2D();

    // ---------------------- Solve a forward kinematics first:
    ccd::RobotStatePtr robot_state_1 = 
        std::make_shared<ccd::RobotState>(robot_model);

    std::vector<double> fk_joints_values_1 = {M_PI_4, 0, 0};
    std::vector<int> joints_numbers_1 = {0,1,2};
    robot_state_1->updateState(fk_joints_values_1, joints_numbers_1);

    robot_state_1->printState("CCD - first configuration", std::vector<int>{0,1,2});
    
    // The length of the links from makeSimpleRobot2D is 1 and in local x direction
    Eigen::Vector4d end_effector_1_O = robot_state_1->getPoint1();
    Eigen::Vector4d end_effector_1_X = robot_state_1->getPoint2();
    Eigen::Vector4d end_effector_1_Y = robot_state_1->getPoint3();

    Eigen::Vector4d expected_1_O(3 * std::cos(M_PI_4),
                               3 * std::sin(M_PI_4),
                               0,
                               1);
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(end_effector_1_O(k) - expected_1_O(k)) < TRANSLATION_ERROR);
    }

    Eigen::Vector4d expected_1_X((3 + 0.2) * std::cos(M_PI_4) ,
                               (3 + 0.2) * std::sin(M_PI_4),
                               0,
                               1);
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(end_effector_1_X(k) - expected_1_X(k)) < TRANSLATION_ERROR);
    }

    Eigen::Vector4d expected_1_Y(3 * std::cos(M_PI_4) - 0.2 * std::sin(M_PI_4),
                                 3 * std::sin(M_PI_4) + 0.2 * std::cos(M_PI_4),
                               0,
                               1);
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(end_effector_1_Y(k) - expected_1_Y(k)) < TRANSLATION_ERROR);
    }

    // -------------------- check if the function getPointXInLinkStartFrame works or not:
    // --- link 2
    // point 1 in start frame of link 2:
    Eigen::Vector4d point1_in_link2 =  robot_state_1->getPoint1InLinkStartFrame(2);
    Eigen::Vector4d point1_in_link2_expected; point1_in_link2_expected << 1,0,0,1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(point1_in_link2(k) - point1_in_link2_expected(k)) < TRANSLATION_ERROR);
    }

    // point 2 in start frame of link 2:
    Eigen::Vector4d point2_in_link2 =  robot_state_1->getPoint2InLinkStartFrame(2);
    Eigen::Vector4d point2_in_link2_expected; point2_in_link2_expected << 1.2,0,0,1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(point2_in_link2(k) - point2_in_link2_expected(k)) < TRANSLATION_ERROR);
    }

    // point 3 in start frame of link 2:
    Eigen::Vector4d point3_in_link2 =  robot_state_1->getPoint3InLinkStartFrame(2);
    Eigen::Vector4d point3_in_link2_expected; point3_in_link2_expected << 1, 0.2, 0, 1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(point3_in_link2(k) - point3_in_link2_expected(k)) < TRANSLATION_ERROR);
    }

    // --- link 1
    // point 1 in start frame of link 1:
    Eigen::Vector4d point1_in_link1 =  robot_state_1->getPoint1InLinkStartFrame(1);
    Eigen::Vector4d point1_in_link1_expected; point1_in_link1_expected << 2,0,0,1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(point1_in_link1(k) - point1_in_link1_expected(k)) < TRANSLATION_ERROR);
    }

    // point 2 in start frame of link 1:
    Eigen::Vector4d point2_in_link1 =  robot_state_1->getPoint2InLinkStartFrame(1);
    Eigen::Vector4d point2_in_link1_expected; point2_in_link1_expected << 2.2,0,0,1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(point2_in_link1(k) - point2_in_link1_expected(k)) < TRANSLATION_ERROR);
    }

    // point 3 in start frame of link 1:
    Eigen::Vector4d point3_in_link1 =  robot_state_1->getPoint3InLinkStartFrame(1);
    Eigen::Vector4d point3_in_link1_expected; point3_in_link1_expected << 2, 0.2, 0, 1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(point3_in_link1(k) - point3_in_link1_expected(k)) < TRANSLATION_ERROR);
    }

    // ---------------------- Solve another forward kinematics close to the first one:
    ccd::RobotStatePtr robot_state_2 = 
        std::make_shared<ccd::RobotState>(robot_model);
    
    double theta_1 = M_PI_4 + 0.1;
    double theta_2 = 0.1;
    double theta_3 = 0.1;
    std::vector<double> fk_joints_values_2 = {theta_1, theta_2, theta_3};
    std::vector<int> joints_numbers_2 = {0,1,2};
    for (int k = 0; k < 3; ++k)
        robot_state_2->updateState(fk_joints_values_2, joints_numbers_2);

    robot_state_2->printState("ccd - second configuration", std::vector<int>{0,1,2});

    Eigen::Vector4d end_effector_2_O = robot_state_2->getPoint1();
    Eigen::Vector4d end_effector_2_X = robot_state_2->getPoint2();
    Eigen::Vector4d end_effector_2_Y = robot_state_2->getPoint3();

    std::cout << "state2_1:\n" << end_effector_2_O << std::endl;
    std::cout << "state2_2:\n" << end_effector_2_X << std::endl;
    std::cout << "state2_3:\n" << end_effector_2_Y << std::endl;

    // first point is on the origin of the end frame of the last link
    Eigen::Vector4d expected_2_O(std::cos(theta_1) + std::cos(theta_1 + theta_2) + std::cos(theta_1 + theta_2 + theta_3),
                                 std::sin(theta_1) + std::sin(theta_1 + theta_2) + std::sin(theta_1 + theta_2 + theta_3),
                                 0,
                                 1);
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(end_effector_2_O(k) - expected_2_O(k)) < TRANSLATION_ERROR);
    }

    // second point is on the x axis of the end frame of the last link by length 0.2
    Eigen::Vector4d expected_2_X(std::cos(theta_1) + std::cos(theta_1 + theta_2) + (1 + 0.2 ) * std::cos(theta_1 + theta_2 + theta_3),
                                 std::sin(theta_1) + std::sin(theta_1 + theta_2) + (1 + 0.2 ) * std::sin(theta_1 + theta_2 + theta_3),
                                  0,
                                  1);
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(end_effector_2_X(k) - expected_2_X(k)) < TRANSLATION_ERROR);
    }

    // third point is on the y axis of the end frame of the last link by length 0.2
    Eigen::Vector4d expected_2_Y(std::cos(theta_1) + std::cos(theta_1 + theta_2) + std::cos(theta_1 + theta_2 + theta_3) - 0.2 * std::sin(theta_1 + theta_2 + theta_3),
                                 std::sin(theta_1) + std::sin(theta_1 + theta_2) + std::sin(theta_1 + theta_2 + theta_3) + 0.2 * std::cos(theta_1 + theta_2 + theta_3),
                                  0,
                                  1);
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(end_effector_2_Y(k) - expected_2_Y(k)) < TRANSLATION_ERROR);
    }

    // -------------------- check if the function getPointXInLinStartFrame works or not:
    // point 1 in start frame of link 1:
    Eigen::Vector4d state2_point1_in_link1 =  robot_state_2->getPoint1InLinkStartFrame(1);
    Eigen::Vector4d state2_point1_in_link1_expected; state2_point1_in_link1_expected << 1 + 1 * std::cos(0.1),
                                                                                        0 + 1 * std::sin(0.1),
                                                                                        0,
                                                                                        1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(state2_point1_in_link1(k) - state2_point1_in_link1_expected(k)) < TRANSLATION_ERROR);
    }

    // point 2 in start frame of link 1:
    Eigen::Vector4d state2_point2_in_link1 =  robot_state_2->getPoint2InLinkStartFrame(1);
    Eigen::Vector4d state2_point2_in_link1_expected; state2_point2_in_link1_expected << 1 + 1.2 * std::cos(0.1),
                                                                                        0 + 1.2 * std::sin(0.1),
                                                                                        0,
                                                                                        1;
    for(int k = 0; k < 4; ++k)
    {
        std::cout << state2_point2_in_link1(k) << "    " << state2_point2_in_link1_expected(k) << std::endl; 
        BOOST_TEST(abs(state2_point2_in_link1(k) - state2_point2_in_link1_expected(k)) < TRANSLATION_ERROR);
    }

    // point 3 in start frame of link 1:
    Eigen::Vector4d state2_point3_in_link1 =  robot_state_2->getPoint3InLinkStartFrame(1);
    Eigen::Vector4d state2_point3_in_link1_expected; state2_point3_in_link1_expected << 1 + 1 * std::cos(0.1) - 0.2 * std::sin(0.1),
                                                                                        0 + 1 * std::sin(0.1) + 0.2 * std::cos(0.1),
                                                                                        0,
                                                                                        1;
    for(int k = 0; k < 4; ++k)
    {
        BOOST_TEST(abs(state2_point3_in_link1(k) - state2_point3_in_link1_expected(k)) < TRANSLATION_ERROR);
    }

    // ---------------- calculate reach for joint number 2
    int joint_number = 2;
    Eigen::Vector4d oe =  robot_state_1->getPoint1InLinkStartFrame(joint_number);
    Eigen::Vector4d xe =  robot_state_1->getPoint2InLinkStartFrame(joint_number);
    Eigen::Vector4d ze =  robot_state_1->getPoint3InLinkStartFrame(joint_number);

    Eigen::Vector4d ote =  expected_2_O.transpose() * robot_state_1->getFrames(joint_number - 1).second.matrix();
    Eigen::Vector4d xte =  expected_2_X.transpose() * robot_state_1->getFrames(joint_number - 1).second.matrix();
    Eigen::Vector4d zte =  expected_2_Y.transpose() * robot_state_1->getFrames(joint_number - 1).second.matrix();
    
    ccd::CalculatorPtr calculator(new ccd::PositionBasedCalculator());
    // the answer will be a set of real numbers representing the solutin angles
    std::vector<double> reaching_angles = calculator->calculateReach(oe, xe, ze, ote, xte, zte);

    for (auto angle : reaching_angles)
    {
        if(angle < 0)
        {
            std::cout << "angle: " << 2 * M_PI + angle << std::endl;
        }else
        {
            std::cout << "angle: " << angle << std::endl;
        }
    }

    // ------------------ Find the min angle of the last joint to minimize the objective function 
    // As I am looking for the joint angle of the last link for minimizing the objective function, I give values
    // to this joint incrementally and calculate the objctive function 
    ccd::RobotStatePtr robot_state_current = 
        std::make_shared<ccd::RobotState>(robot_model);
    
    std::vector<Eigen::Vector4d> target_3points = {expected_2_O, expected_2_X, expected_2_Y};
    double min_value = 1000;
    double min_joint_value;
    for(double jv = 0; jv < 2 * M_PI; jv += 0.05)
    {
        std::vector<double> fk_joints_values_current = {0, 0, jv};
        std::vector<int> joints_numbers_current = {0, 1, 2};
        robot_state_current->updateState(fk_joints_values_current, joints_numbers_current);

        double value = objectiveFunction(robot_state_current, target_3points);
        // std::cout << "joint value: " << jv << " ---- objective function value: " << value << std::endl;
        if (value < min_value)
        {
            min_value = value;
            min_joint_value = jv;
        }
    }

    std::cout << " min value ===>>> " << min_value << std::endl;
    std::cout << " min joint value ===>>> " << min_joint_value << std::endl;


    double of_value = objectiveFunction(robot_state_1, target_3points);
    std::cout << "objective function value for state1: " << of_value << std::endl;

    // --------------------- get the end effector frame of the second state
    // Eigen::Affine3d target = end_effector_2;
    // double threshold = 0.01;
    // double requested_iteration_num = 100;

    // ccd::CCDPtr ccd(new ccd::CCD(robot_model, fk_joints_values_1));

    // ccd->setInverseKinematicsInput(target,
    //                                  threshold,
    //                                  requested_iteration_num,
    //                                  ccd::CalculatorType::POSITION);
    
    // ccd::IKOutput output;
    // bool solved = ccd->solveIK(output);

    // if(solved)
    // {
    //     std::cout << "solveIK was successful" << std::endl;
    //     std::cout << "total iteration: " << output.final_iteration_num << std::endl;
    //     std::cout << "error: " << output.target_ee_error << std::endl;
    // }
}

BOOST_AUTO_TEST_CASE(CCD3D)
{
   

}

