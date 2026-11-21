/*
#    Copyright (c) 2024-2026 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Daniel S. J. Derwent (email: daniel.derwent@manchester.ac.uk)
#
#   Acknowledgement: This file was adapted from the file UnitreeG1Robot.cpp
#                    originally written by Juan Jose Quiroz Omana 
#                    (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

#include<cmath>
#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/UnitreeH1Robot.h>

namespace DQ_robotics
{

const double shoulder_tilt = 0.43633;
const double forearm_length = 0.270; // Estimated based on measurements of STL file

// Internal helper functions are static (internal linkage) so they are not exported from the
// shared library and cannot clash with same-named symbols in other files or libraries.
static DQ _get_arm_base_offset(const std::string left_or_right);
static DQ _get_leg_base_offset(const std::string left_or_right);
static DQ_SerialManipulatorDH _get_arm(const std::string left_or_right);
static DQ_SerialManipulatorDH _get_leg(const std::string left_or_right);
static DQ_SerialManipulatorDH _get_h1_waist_chain();
static DQ_SerialManipulatorDH _get_h1_left_leg_chain();
static DQ_SerialManipulatorDH _get_h1_right_leg_chain();
static DQ_SerialManipulatorDH _get_h1_right_arm_chain();
static DQ_SerialManipulatorDH _get_h1_left_arm_chain();

DQ_SerialManipulatorDH UnitreeH1Robot::kinematics(const LIMB& limb)
{
    switch (limb) {

    case LIMB::LEFT_ARM:
        return _get_h1_left_arm_chain();
    case LIMB::RIGHT_ARM:
        return _get_h1_right_arm_chain();
    case LIMB::WAIST:
        return _get_h1_waist_chain();
    case LIMB::LEFT_LEG:
        return _get_h1_left_leg_chain();
    case LIMB::RIGHT_LEG:
        return _get_h1_right_leg_chain();
    default:
        throw std::runtime_error("Invalid limb");
    }
}

// Helper functions to get arm and leg parameters.
static DQ _get_arm_base_offset(const std::string left_or_right)
{
    int side_sign;
    if(left_or_right == "left"){
        side_sign = 1;
    }
    else if(left_or_right == "right"){
        side_sign = -1;
    }
    else{
        throw std::runtime_error("Invalid left_or_right, must be 'left' or 'right'.");
    }
    
    // Align DH z_0 with the URDF shoulder-pitch axis (+y).
    const double angle = side_sign * shoulder_tilt - M_PI / 2.0;
    const DQ rotation = std::cos(angle / 2.0) + i_ * std::sin(angle / 2.0);
    const DQ translation(0.0, 0.0055, side_sign * 0.15535, 0.42999);
    return rotation + E_ * 0.5 * translation * rotation;
}

static DQ_SerialManipulatorDH _get_arm(const std::string left_or_right)
{
    int side_sign;
    Eigen::Vector4d lower, upper;
    if(left_or_right == "left"){
        side_sign = 1;
        lower << -2.87, -0.34, -1.30, -1.25;
        upper <<  2.87,  3.11,  4.45,  2.61;
    }
    else if(left_or_right == "right"){
        side_sign = -1;
        lower << -2.87, -3.11, -4.45, -1.25;
        upper <<  2.87,  0.34,  1.30,  2.61;
    }
    else{
        throw std::runtime_error("Invalid left_or_right, must be 'left' or 'right'.");
    }

    // Columns: shoulder pitch, shoulder roll, shoulder yaw, elbow.
    // The final DH frame coincides with the end effector frame.
    Eigen::Matrix<double, 5, 4> dh;
    dh << M_PI / 2.0,         -M_PI / 2.0 - side_sign * shoulder_tilt, -M_PI / 2.0,         M_PI,          //theta
         side_sign * 0.0565, -0.0055,                                 -(0.1343 + 0.198),   0.0,            //d
         0.0165,              0.0,                                    -0.0185,             forearm_length, //a
         M_PI / 2.0,          M_PI / 2.0,                              M_PI / 2.0,         M_PI / 2.0,     //alpha
         0.0,                 0.0,                                     0.0,                0.0;            //Type of joint. 0= Revolute, 1= Prismatic

    auto arm = DQ_SerialManipulatorDH(dh);
    arm.set_lower_q_limit(lower);
    arm.set_upper_q_limit(upper);
    return arm;
}

static DQ _get_leg_base_offset(const std::string left_or_right)
{
    int side_sign;
    if(left_or_right == "left"){
        side_sign = 1;
    }
    else if(left_or_right == "right"){
        side_sign = -1;
    }
    else{
        throw std::runtime_error("Invalid left_or_right, must be 'left' or 'right'.");
    }

    // Align the DH base frame with the hip-yaw axis and place it at the URDF joint origin.
    const DQ rotation = std::cos(M_PI / 4.0) + k_ * std::sin(M_PI / 4.0);
    const DQ translation(0.0, 0.0, side_sign * 0.0875, -0.1742);
    return rotation + E_ * 0.5 * translation * rotation;
}

static DQ_SerialManipulatorDH _get_leg(const std::string left_or_right)
{
    int side_sign;
    Eigen::Matrix<double, 5, 1> lower, upper;
    if(left_or_right == "left"){
        side_sign = 1;
        lower << -0.43, -0.43, -3.14, -0.26, -0.87;
        upper <<  0.43,  0.43,  2.53,  2.05,  0.52;
    }
    else if(left_or_right == "right"){
        side_sign = -1;
        lower << -0.43, -0.43, -3.14, -0.26, -0.87;
        upper <<  0.43,  0.43,  2.53,  2.05,  0.52;
    }
    else{
        throw std::runtime_error("Invalid left_or_right, must be 'left' or 'right'.");
    }

    // Columns: hip yaw, hip roll, hip pitch, knee, ankle.
    // The final DH frame coincides with the URDF ankle-link frame.
    Eigen::Matrix<double, 5, 5> dh;
    dh << 0.0,        M_PI / 2.0, M_PI,                0.0, -M_PI / 2.0, // theta
          0.0,        0.039468,   side_sign * 0.11536, 0.0,  0.0,        // d
          0.0,        0.0,        0.4,                 0.4,  0.0,        // a
          M_PI / 2.0, M_PI / 2.0, 0.0,                 0.0,  M_PI / 2.0, // alpha
          0.0,        0.0,        0.0,                 0.0,  0.0;        // Type of joint. 0= Revolute, 1= Prismatic

    auto leg = DQ_SerialManipulatorDH(dh);
    leg.set_lower_q_limit(lower);
    leg.set_upper_q_limit(upper);
    return leg;
}

// Limb getter functions
static DQ_SerialManipulatorDH _get_h1_waist_chain()
{
    Eigen::Matrix<double, 5, 1> waist_dh;
    waist_dh << 0.0, 0.0, 0.0, 0.0, 0.0;

    DQ_SerialManipulatorDH h1_waist(waist_dh);

    Eigen::VectorXd lower(1), upper(1);
    lower << -2.35;
    upper <<  2.35;
    h1_waist.set_lower_q_limit(lower);
    h1_waist.set_upper_q_limit(upper);

    return h1_waist;
}

static DQ_SerialManipulatorDH _get_h1_left_leg_chain()
{
    DQ_SerialManipulatorDH h1_left_leg = _get_leg("left");
    h1_left_leg.set_reference_frame(_get_leg_base_offset("left"));
    h1_left_leg.set_base_frame(_get_leg_base_offset("left"));

    return h1_left_leg;
}

static DQ_SerialManipulatorDH _get_h1_right_leg_chain()
{
    DQ_SerialManipulatorDH h1_right_leg = _get_leg("right");
    h1_right_leg.set_reference_frame(_get_leg_base_offset("right"));
    h1_right_leg.set_base_frame(_get_leg_base_offset("right"));

    return h1_right_leg;
}

static DQ_SerialManipulatorDH _get_h1_right_arm_chain()
{
    DQ_SerialManipulatorDH h1_right_arm = _get_arm("right");
    h1_right_arm.set_reference_frame(_get_arm_base_offset("right"));
    h1_right_arm.set_base_frame(_get_arm_base_offset("right"));

    return h1_right_arm;
}

static DQ_SerialManipulatorDH _get_h1_left_arm_chain()
{
    DQ_SerialManipulatorDH h1_left_arm = _get_arm("left");
    h1_left_arm.set_reference_frame(_get_arm_base_offset("left"));
    h1_left_arm.set_base_frame(_get_arm_base_offset("left"));

    return h1_left_arm;
}

}
