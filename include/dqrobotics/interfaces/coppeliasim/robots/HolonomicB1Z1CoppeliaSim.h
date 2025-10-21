/*
#    Copyright (c) 2024-2025 Adorno-Lab
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
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
#include "dqrobotics/robots/UnitreeB1Z1MobileRobot.h"

namespace DQ_robotics
{
class HolonomicB1Z1CoppeliaSim: public DQ_CoppeliaSimRobotZMQ
{
private:
    std::string robot_name_;
    std::vector<std::string> arm_jointnames_;
    std::vector<std::string> base_jointnames_;
    VectorXd _get_mobile_robot_configuration_from_pose(const DQ &base) const;
    std::shared_ptr<UnitreeB1Z1MobileRobot> robot_model_;

public:
    HolonomicB1Z1CoppeliaSim(const std::string& robot_name,
                             const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& coppeliasim_interface_sptr);

    void set_configuration(const VectorXd& configuration) override;
    void set_target_configuration(const VectorXd& target_configuration) override;
    void set_target_configuration_velocities(const VectorXd& target_configuration_velocities) override; //
    void set_target_configuration_forces(const VectorXd& target_configuration_forces) override;
    VectorXd get_configuration() override;
    VectorXd get_configuration_velocities() override;
    VectorXd get_configuration_forces() override;

    void set_target_holonomic_velocities(const VectorXd& u_base);
    void set_arm_joint_target_positions(const VectorXd& q_arm);

    VectorXd get_planar_joint_velocities_at_body_frame(const VectorXd& qbase,
                const VectorXd &planar_joint_velocities_at_inertial_frame) const;

    std::shared_ptr<DQ_Kinematics> get_model() const;
};
}
