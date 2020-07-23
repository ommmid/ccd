#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Robot Model"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/world/world.h"
#include "fabrik/util/exception.h"
#include "fabrik/robot_model/robot_model.h"


BOOST_AUTO_TEST_SUITE(Suite1)

BOOST_AUTO_TEST_CASE(EmptyChain)
{
    std::cout << "========== Empty Chain" << std::endl;
    std::vector<fabrik::Link> chain;
    const Eigen::Affine3d& base = Eigen::Affine3d::Identity();
    
    BOOST_CHECK_THROW(fabrik::RobotModel("test_robot", base, chain), std::exception);
}


BOOST_AUTO_TEST_SUITE_END()
