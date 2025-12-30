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

#include <dqrobotics_extensions/robot_constraint_manager/numpy.hpp>

#include "dqrobotics/robots/UnitreeB1Z1MobileRobot.h"
#include "dqrobotics/robots/UnitreeZ1Robot.h"

namespace DQ_robotics
{

UnitreeB1Z1MobileRobot::UnitreeB1Z1MobileRobot()
{
    kin_arm_ = std::make_shared<DQ_SerialManipulatorDH>(UnitreeZ1Robot::kinematics());
    kin_holonomic_base_ = std::make_shared<DQ_HolonomicBase>();
    kin_holonomic_base_->set_frame_displacement(DQ(1));

    Is_ = (MatrixXd(3,8) << 0,0,0,1, 0,0,0,0,
                            0,0,0,0, 0,1,0,0,
                            0,0,0,0, 0,0,1,0).finished();

    zero_3x6_ = MatrixXd::Zero(3,6);

    zero_6x3_ = MatrixXd::Zero(6,3);
    I6x6_ = MatrixXd::Identity(6,6);
}

DQ UnitreeB1Z1MobileRobot::fkm(const VectorXd &q, const int &ith) const
{
    VectorXd qbase = q.head(3);
    VectorXd qarm  = q.tail(6);
    DQ x;
    if (ith < 2 or ith > 8)
        throw std::runtime_error("UnitreeB1Z1MobileRobot::fkm: The minimum index is 2. The maximum index is 8. "
                                 "However, you used "+ std::to_string(ith));
    else if (ith == 2)
    {
        x = kin_holonomic_base_->fkm(qbase)*X_HEIGHT_OFFSET_;
    }else
    {
        x = kin_holonomic_base_->fkm(qbase)*X_HEIGHT_OFFSET_*X_J1_OFFSET_*kin_arm_->fkm(qarm, ith-3);
    }
    return x;

}

DQ UnitreeB1Z1MobileRobot::fkm(const VectorXd &q) const
{
    return fkm(q, get_dim_configuration_space()-1);
}

MatrixXd UnitreeB1Z1MobileRobot::pose_jacobian(const VectorXd &q, const int &ith) const
{
    VectorXd qbase = q.head(3);
    VectorXd qarm  = q.tail(6);

    MatrixXd J;
    if (ith < 2 or ith > 8)
        throw std::runtime_error("UnitreeB1Z1MobileRobot::pose_jacobian: The minimum index is 2. The maximum index is 8. "
                                 "However, you used "+ std::to_string(ith));
    else if (ith == 2)
    {
        MatrixXd Jaux = haminus8(X_HEIGHT_OFFSET_)*kin_holonomic_base_->pose_jacobian(qbase);
        J = DQ_robotics_extensions::Numpy::resize(Jaux, 8, get_dim_configuration_space());
    }else
    {
        MatrixXd J1 = haminus8(X_HEIGHT_OFFSET_*X_J1_OFFSET_*kin_arm_->fkm(qarm))*kin_holonomic_base_->pose_jacobian(qbase);
        MatrixXd J2 = hamiplus8(kin_holonomic_base_->fkm(qbase)*X_HEIGHT_OFFSET_*X_J1_OFFSET_)*kin_arm_->pose_jacobian(qarm, ith-3);
        J = DQ_robotics_extensions::Numpy::hstack(J1, J2);
        if (J.cols() != 9)
            J = DQ_robotics_extensions::Numpy::resize(J, 8, get_dim_configuration_space());
    }

    return J;
}

MatrixXd UnitreeB1Z1MobileRobot::pose_jacobian(const VectorXd &q) const
{
    return pose_jacobian(q, get_dim_configuration_space()-1);
}

int UnitreeB1Z1MobileRobot::get_dim_configuration_space() const
{
    return dim_configuration_space_;
}

MatrixXd UnitreeB1Z1MobileRobot::pose_jacobian_derivative([[maybe_unused]] const VectorXd &q,
                                                          [[maybe_unused]] const VectorXd &q_dot,
                                                          [[maybe_unused]] const int &to_ith_link) const
{
    throw std::runtime_error("pose_jacobian_derivative is not implemented yet.");
}

MatrixXd UnitreeB1Z1MobileRobot::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    return pose_jacobian_derivative(q, q_dot, get_dim_configuration_space()-1);
}

void UnitreeB1Z1MobileRobot::update_base_offset(const DQ &X_J1_OFFSET)
{
    X_J1_OFFSET_ = X_J1_OFFSET;
}

void UnitreeB1Z1MobileRobot::update_base_height_from_IMU(const DQ &X_IMU)
{
    VectorXd p_IMU = X_IMU.translation().vec3();
    X_HEIGHT_OFFSET_ = 1+0.5*E_*p_IMU(2)*k_;
}


/**
 * @brief UnitreeB1Z1MobileRobot::compute_saturation_constraints
 * @param q The robot configuration
 * @param q_dot_limits The desired saturation limits
 * @return A tuple {A,b} containing the saturation constraints
 */
std::tuple<MatrixXd, VectorXd>
UnitreeB1Z1MobileRobot::compute_saturation_constraints(const VectorXd& q,
                                                       const std::tuple<VectorXd, VectorXd>& q_dot_limits)
{
    MatrixXd Alim(18,9);
    VectorXd blim(18);
    MatrixXd Jhol =  pose_jacobian(q,2);
    MatrixXd Jtwist_b =  Is_*2*hamiplus8(fkm(q,2).conj())*Jhol.block(0,0,8,3);

    const auto [q_dot_min, q_dot_max] = q_dot_limits;

    MatrixXd part1(3,9);
    part1 << Jtwist_b, zero_3x6_;

    MatrixXd part2(3,9);
    part2 << -Jtwist_b, zero_3x6_;

    MatrixXd part3(6,9);
    part3 << zero_6x3_, I6x6_;

    MatrixXd part4(6,9);
    part4 << zero_6x3_, -I6x6_;

    Alim <<part1, part2, part3, part4;

    blim << q_dot_max(2),  q_dot_max(0),  q_dot_max(1),
           -q_dot_min(2), -q_dot_min(0), -q_dot_min(1),
            q_dot_max.tail(6),
           -q_dot_min.tail(6);

    return {Alim, blim};
}


}
