#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Robot State"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/world/world.h"
#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"
#include "fabrik/robot_model/robot_model.h"
#include "fabrik/robot_state/robot_state.h"

const double TRANSLATION_ERROR = 1e-3;
const double ANGLE_ERROR = 1e-3;

class MakerSpatial
{
public:

    MakerSpatial()
    {
        makeChain();
        makeBase();
    }

    void makeChain()
    {
        Eigen::Vector3d vec1(5,1,3);
        vec1.normalize();
        Eigen::Affine3d link1_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec1));
        link1_frame.translation() = Eigen::Vector3d(1,2.2,3);
        fabrik::Link link1("link1",  link1_frame);

        Eigen::Vector3d vec2(3,2,3);
        vec2.normalize();
        Eigen::Affine3d link2_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec2));
        link2_frame.translation() = Eigen::Vector3d(1.7, 3, 2.5);
        fabrik::Link link2("link2",  link2_frame);

        Eigen::Vector3d vec3(1,6,4);
        vec3.normalize();
        Eigen::Affine3d link3_frame(Eigen::AngleAxisd(fabrik::randomDouble(0, 1), vec3));
        link3_frame.translation() = Eigen::Vector3d(0.5, 4, 1);
        fabrik::Link link3("link3",  link3_frame);

        chain.push_back(link1);
        chain.push_back(link2);
        chain.push_back(link3);
    }

    void makeBase()
    {
        Eigen::Vector3d vec0(0,1,1);
        vec0.normalize();
        Eigen::Affine3d base_transformation(Eigen::AngleAxisd(0, vec0));
        base_transformation.translate(Eigen::Vector3d(0.5, 0.5, 0.5));
        base = base_transformation;
    }

    std::vector<fabrik::Link> chain;
    Eigen::Affine3d base;
};


BOOST_AUTO_TEST_CASE(RobotStateConstructor)
{
    fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot2D();

    fabrik::RobotStatePtr robot_state = 
        std::make_shared<fabrik::RobotState>(robot_model);
     
    robot_state->printState("Robot State Constructor", std::vector<int>{-1});
}

BOOST_AUTO_TEST_CASE(updateStateTarget)
{
    std::cout << "========== Robot State Target ==========" << std::endl;

    fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot2D();
    fabrik::RobotStatePtr robot_state = 
        std::make_shared<fabrik::RobotState>(robot_model);

    // check updateState for setting target at FORWARD reaching
    robot_state->setReachingDirection(fabrik::ReachingDirection::FORWARD);
    Eigen::Affine3d target(Eigen::AngleAxisd(M_PI/7, Eigen::Vector3d::UnitZ()));

    BOOST_CHECK_THROW(robot_state->updateState(target), std::exception);
}

BOOST_AUTO_TEST_CASE(updateState)
{
    fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot2D();
    fabrik::RobotStatePtr robot_state = 
        std::make_shared<fabrik::RobotState>(robot_model);

    robot_state->printState("update state test - initial - forward completed", std::vector<int>{-1});

    // one backward reaching
    robot_state->setReachingDirection(fabrik::ReachingDirection::BACKWARD);

    robot_state->updateState(0.3, 2);
    robot_state->printState("update state test - update J_2 - backward", std::vector<int>{-1});

    robot_state->updateState(0.4, 1);
    robot_state->printState("update state test - update J_1 - backward", std::vector<int>{-1});

    BOOST_CHECK_THROW(robot_state->updateState(0.5, 0), std::exception);
    robot_state->printState("update state test - update J_0 - backward", std::vector<int>{-1});

    // one forward reaching
    robot_state->setReachingDirection(fabrik::ReachingDirection::FORWARD);

    robot_state->updateState(0.6, 0);
    robot_state->printState("update state test - update J_0 - forward", std::vector<int>{-1});

    robot_state->updateState(0.45, 1);
    robot_state->printState("update state test - update J_1 - forward", std::vector<int>{-1});

    robot_state->updateState(0.26, 2);
    robot_state->printState("update state test - update J_2 - forward", std::vector<int>{-1});
}

BOOST_AUTO_TEST_CASE(ForwardKinematics2D)
{
    // Solve a simple 2D forward kinematics :
    fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot2D();
    fabrik::RobotStatePtr robot_state = 
        std::make_shared<fabrik::RobotState>(robot_model);
    
    robot_state->setReachingDirection(fabrik::ReachingDirection::FORWARD);
    std::vector<double> fk_joints_values = {M_PI_4, 0, 0};

    for (int k = 0; k < 3; ++k)
        robot_state->updateState(fk_joints_values[k], k);
    
    Eigen::Affine3d end_effector = robot_state->getFrames(2).second;

    // robot_state->printState("test/fabrik.cpp : FK 2D", std::vector<int>{-1});

    // The location of the last frame (end_effector) should be at:
    Eigen::Vector3d expected_location(3 * std::cos(M_PI_4),
                                      3 * std::sin(M_PI_4),
                                      0);

    for(int k = 0; k < 3; ++k)
    {
        BOOST_TEST(abs(end_effector.translation()[k] - expected_location(k)) < TRANSLATION_ERROR);
    }
}

