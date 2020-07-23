/* Author: Omid Heidari */

#include <iostream>
#include <Eigen/Geometry>

#include "ccd/robot_model/robot_model.h"

#include "ccd/util/exception.h"


namespace ccd
{

// -------------------- Link --------------------
Link::Link(const std::string& link_name, const Eigen::Affine3d& link_frame):
link_name_(link_name), link_frame_(link_frame)
{
}

Link::Link(const std::string& link_name, const Eigen::Affine3d& start_frame, const Eigen::Affine3d& end_frame):
link_name_(link_name)
{
    link_frame_ = start_frame.inverse() * end_frame;
}

// // -------------------- Chain --------------------
RobotModel::RobotModel(const std::string& robot_name, const Eigen::Affine3d& base, const std::vector<Link>& chain):
robot_name_(robot_name), base_(base), chain_(chain)
{   
    int dof = (int)chain_.size();

    if( dof == 0)
    {
         throw ccd::Exception("a chain can not be created by 0 link");
    } 
    else
    {
        dof_ = dof;
    }
}

RobotModelPtr makeSimpleRobot2D()
{
    std::string robot_name = "simple 2D robot";
    std::vector<ccd::Link> chain;

    Eigen::Vector3d vec1(0,0,1);
    vec1.normalize();
    Eigen::Affine3d link1_frame(Eigen::AngleAxisd(0, vec1));
    link1_frame.translation() = Eigen::Vector3d(1, 0, 0);
    ccd::Link link1("link1",  link1_frame);

    Eigen::Vector3d vec2(0,0,1);
    vec2.normalize();
    Eigen::Affine3d link2_frame(Eigen::AngleAxisd(0, vec2));
    link2_frame.translation() = Eigen::Vector3d(1, 0, 0);
    ccd::Link link2("link2",  link2_frame);

    Eigen::Vector3d vec3(0,0,1);
    vec3.normalize();
    Eigen::Affine3d link3_frame(Eigen::AngleAxisd(0, vec3));
    link3_frame.translation() = Eigen::Vector3d(1, 0, 0);
    ccd::Link link3("link3",  link3_frame);

    chain.push_back(link1);
    chain.push_back(link2);
    chain.push_back(link3);
    
    Eigen::Vector3d vec0(0,0,1);
    vec0.normalize();
    Eigen::Affine3d base(Eigen::AngleAxisd(0, vec0));
    base.translation() = Eigen::Vector3d(0, 0, 0);

    RobotModelPtr robot_model = std::make_shared<RobotModel>(robot_name, base, chain);
    return robot_model;
}

RobotModelPtr makeSimpleRobot3D()
{
    std::string robot_name = "simple 3D robot";

    Eigen::Vector3d vec0(0,0,1);;
    vec0.normalize();
    Eigen::Affine3d base(Eigen::AngleAxisd(0, vec0));
    base.translation() = Eigen::Vector3d(0, 0, 0);
    std::cout << "base_frame:\n" << base.matrix() << std::endl;

    Eigen::Vector3d vec1 = Eigen::Vector3d(1,2,3); vec1.normalize();
    Eigen::Affine3d link1_frame(Eigen::AngleAxisd(0.2, vec1));
    Eigen::Vector3d trans1 =  Eigen::Vector3d(1,1,0.3); trans1.normalize(); 
    link1_frame.translation() = trans1;
    ccd::Link link1("link1",  link1_frame);
    std::cout << "link1_frame:\n" << link1_frame.matrix() << std::endl;

    Eigen::Vector3d vec2 = Eigen::Vector3d(1,1,3); vec2.normalize();
    Eigen::Affine3d link2_frame(Eigen::AngleAxisd(0.6, vec2));
    Eigen::Vector3d trans2 = Eigen::Vector3d(2,1,0.5); trans2.normalize();
    link2_frame.translation() = trans2;
    ccd::Link link2("link2",  link2_frame);
    std::cout << "link2_frame:\n" << link2_frame.matrix() << std::endl;

    Eigen::Vector3d vec3 = Eigen::Vector3d(2,1,4); vec3.normalize();
    Eigen::Affine3d link3_frame(Eigen::AngleAxisd(0.7, vec3));
    Eigen::Vector3d trans3 = Eigen::Vector3d(3,1,0.7); trans3.normalize();
    link3_frame.translation() = trans3;
    ccd::Link link3("link3",  link3_frame);
    std::cout << "link3_frame:\n" << link3_frame.matrix() << std::endl;

    std::vector<ccd::Link> chain;
    chain.push_back(link1);
    chain.push_back(link2);
    chain.push_back(link3);
    
    ccd::RobotModelPtr robot_model = std::make_shared<ccd::RobotModel>(robot_name, base, chain);
    return robot_model;
}

} // namespace ccd