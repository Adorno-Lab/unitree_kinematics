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
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

#pragma once
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_robotics
{
/**
 * Example of usage (DH parameters derived from Unitree's g1_29dof.urdf):
 *
 *       auto left_arm_robot  = UnitreeG1Robot::kinematics(UnitreeG1Robot::LIMB::LEFT_ARM);
 *       auto right_arm_robot = UnitreeG1Robot::kinematics(UnitreeG1Robot::LIMB::RIGHT_ARM);
 *       auto left_leg_robot  = UnitreeG1Robot::kinematics(UnitreeG1Robot::LIMB::LEFT_LEG);
 *       auto right_leg_robot = UnitreeG1Robot::kinematics(UnitreeG1Robot::LIMB::RIGHT_LEG);
 *       auto waist_robot     = UnitreeG1Robot::kinematics(UnitreeG1Robot::LIMB::WAIST);
 *
 *       VectorXd q_waist     = VectorXd::Zero(3);
 *       VectorXd q_left_arm  = VectorXd::Zero(7);
 *       VectorXd q_right_arm = VectorXd::Zero(7);
 *       VectorXd q_left_leg  = VectorXd::Zero(6);
 *       VectorXd q_right_leg = VectorXd::Zero(6);
 *
 *       // Pelvis pose in the world frame (here: nominal standing height, feet on the ground).
 *       DQ pelvis_frame = 1 + 0.5*E_*0.79227*k_;
 *
 *       DQ x_waist     = pelvis_frame*waist_robot.fkm(q_waist);        // torso_link
 *       DQ x_left_arm  = x_waist*left_arm_robot.fkm(q_left_arm);       // left_rubber_hand (palm)
 *       DQ x_right_arm = x_waist*right_arm_robot.fkm(q_right_arm);     // right_rubber_hand (palm)
 *       DQ x_left_leg  = pelvis_frame*left_leg_robot.fkm(q_left_leg);  // left_ankle_roll_link
 *       DQ x_right_leg = pelvis_frame*right_leg_robot.fkm(q_right_leg);// right_ankle_roll_link
 */
class UnitreeG1Robot
{
public:
    enum class LIMB{LEFT_ARM,
                    RIGHT_ARM,
                    WAIST,
                    LEFT_LEG,
                    RIGHT_LEG};

    static DQ_SerialManipulatorDH kinematics(const LIMB& limb);
};

}
