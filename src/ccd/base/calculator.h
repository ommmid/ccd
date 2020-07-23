/* Author: Omid Heidari */
#pragma once

#include <memory>
#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "ccd/robot_model/robot_model.h"
#include "ccd/util/class_forward.h"
#include "ccd/util/io.h"

namespace ccd
{  
CCD_CLASS_FORWARD(Calculator);
CCD_CLASS_FORWARD(PositionBasedCalculator);


enum CalculatorType
{
    POSITION,
    ORIENTATION,
    COMBINATION
};


/** \brief This class includes the problem input, output and the solver
 */
class Calculator
{
public:
    /** \brief Construct a Calculator object */
    // Calculator();
    
    /** \brief Calculate the error between two affine3d frames */
    virtual double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) = 0;

    /** \brief Calculates the transformation of the reaching frame based on some criteria  */
    virtual std::vector<double>  calculateReach(const Eigen::Vector4d oe, 
                                const Eigen::Vector4d xe,
                                const Eigen::Vector4d ze,
                                const Eigen::Vector4d ote, 
                                const Eigen::Vector4d xte,
                                const Eigen::Vector4d zte) = 0;

protected:

    double error_;


};

class PositionBasedCalculator : public Calculator
{
public:
    PositionBasedCalculator();

    double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) override;

    /** \brief Finds the angle to go FROM start_to_end_projected TO start_to_aim_projected */
    std::vector<double>  calculateReach(  const Eigen::Vector4d oe, 
                            const Eigen::Vector4d xe,
                            const Eigen::Vector4d ze,
                            const Eigen::Vector4d ote, 
                            const Eigen::Vector4d xte,
                            const Eigen::Vector4d zte) override;

    // all these vectors are expressed in world frame
    Eigen::Vector3d start_to_aim;
    Eigen::Vector3d start_to_aim_projected;
    Eigen::Vector3d start_to_end;
    Eigen::Vector3d start_to_end_projected;

private:

};

class OrientationBasedCalculator : public Calculator
{
public:
    OrientationBasedCalculator();

    double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) override;

    std::vector<double>  calculateReach(const Eigen::Vector4d oe, 
                            const Eigen::Vector4d xe,
                            const Eigen::Vector4d ze,
                            const Eigen::Vector4d ote, 
                            const Eigen::Vector4d xte,
                            const Eigen::Vector4d zte) override;

};

class ComboCalculator : public Calculator
{
public:
    ComboCalculator();

    double calculateError(const Eigen::Affine3d& frame_1, const Eigen::Affine3d& frame_2) override;

    std::vector<double>  calculateReach(const Eigen::Vector4d oe, 
                            const Eigen::Vector4d xe,
                            const Eigen::Vector4d ze,
                            const Eigen::Vector4d ote, 
                            const Eigen::Vector4d xte,
                            const Eigen::Vector4d zte) override;

};

}