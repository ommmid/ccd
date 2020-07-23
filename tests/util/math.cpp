#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Math"

#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include <boost/test/unit_test.hpp>

#include "fabrik/util/exception.h"
#include "fabrik/util/math.h"

const double ANGLE_TOLERANCE = 1e-6;

BOOST_AUTO_TEST_CASE(Rotation_Z)
{
    double theta = 0;
    Eigen::Affine3d rotation_z = fabrik::rotation_z(theta);
    Eigen::MatrixXd rot_around_z(4,4);
    rot_around_z << std::cos(theta), -std::sin(theta), 0, 0,
                    std::sin(theta),  std::cos(theta), 0, 0,
                    0,                  0,             1, 0,
                    0,                  0,             0, 1;
    
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            BOOST_CHECK_EQUAL(rotation_z(i,j), rot_around_z(i,j));
        }
    }
}

BOOST_AUTO_TEST_CASE(RotationMatrix)
{
    // check if the inveres of a rotation is the rotation of the negative angle
    // for (int i = 0; i < 4; ++i){
    //     for (int j = 0; j < 4; ++j){
    //         BOOST_CHECK_EQUAL(rotation_z(i,j), rot_around_z(i,j));
    //     }
    // }

    double theta = 0;
    Eigen::AngleAxisd angle_axis(theta, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q(angle_axis);
    Eigen::Matrix3d e_1 = q.matrix();
    std::cout << "e_1:" << std::endl << e_1 << std::endl;

    theta = 30 * M_PI / 180;
    Eigen::AngleAxisd angle_axis_1(theta, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q_1(angle_axis_1);
    Eigen::Matrix3d rotation_matrix_1 = q_1.matrix();
    std::cout << "rotation_matrix_1:" << std::endl << rotation_matrix_1 << std::endl;

    // Multiplication   
    Eigen::Matrix3d s_2 = e_1 * rotation_matrix_1;
    std::cout << "s_2:" << std::endl << s_2 << std::endl;

    theta = -30 * M_PI / 180;
    Eigen::AngleAxisd angle_axis_2(theta, Eigen::Vector3d::UnitZ());
    Eigen::Quaternion<double> q_2(angle_axis_2);
    Eigen::Matrix3d rotation_matrix_2 = q_2.matrix();
    std::cout << "rotation_matrix_2:" << std::endl << rotation_matrix_2 << std::endl;

    // Multiplication
    s_2 = e_1 * rotation_matrix_2;
    std::cout << "s_2:" << std::endl << s_2 << std::endl;



    // Eigen::MatrixXd rot_around_z(4,4);
    // rot_around_z << std::cos(theta), -std::sin(theta), 0, 0,
    //                 std::sin(theta),  std::cos(theta), 0, 0,
    //                 0,                  0,             1, 0,
    //                 0,                  0,             0, 1;
    
    
}

BOOST_AUTO_TEST_CASE(RandomDouble)
{
    double random_double = fabrik::randomDouble(1,5);
    BOOST_TEST(random_double >= 1 );
    BOOST_TEST(random_double <= 5 );
    
}

BOOST_AUTO_TEST_CASE(signedAngleBetweenTwoVectors)
{
    // Angle between a vector and itself
    Eigen::Vector3d v1(5,6,8);
    Eigen::Vector3d v2(5,6,8);
    Eigen::Vector3d n(0,0,1);
    double angle = fabrik::signedAngleBetweenTwoVectors(v1,v2,n);
    std::cout << " ========== angle: " << angle << " ? " << 0 << std::endl;
    BOOST_TEST( std::abs(angle - 0) <= ANGLE_TOLERANCE );    

    // simple positive pi/2 test
    v1 = {5,0,0};
    v2 = {0,3,0};
    angle = fabrik::signedAngleBetweenTwoVectors(v1,v2,n);
    std::cout << " ========== angle: " << angle << " ? " << M_PI_2 << std::endl;
    BOOST_TEST( std::abs(angle - M_PI_2) <= ANGLE_TOLERANCE );    

    // v2 in the second quadrant
    v1 = {5,0,0};
    double theta = 150 * M_PI / 180;
    v2 = {std::cos(theta), std::sin(theta), 0};
    angle = fabrik::signedAngleBetweenTwoVectors(v1,v2,n);
    std::cout << " ========== angle: " << angle << " ? " << theta << std::endl;
    BOOST_TEST( std::abs(angle - theta) <= ANGLE_TOLERANCE );    

    // v2 in the third quadrant
    v1 = {1,0,0};
    theta = 200 * M_PI / 180;
    v2 = {std::cos(theta), std::sin(theta), 0};
    angle = fabrik::signedAngleBetweenTwoVectors(v1,v2,n);
    std::cout << " ========== angle: " << angle << " ? " << theta - 2 * M_PI << std::endl;
    BOOST_TEST( std::abs(angle - (theta - 2 * M_PI)) <= ANGLE_TOLERANCE );    
}

