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

#include "dqrobotics/interfaces/coppeliasim/robots/UnitreeB1Z1CoppeliaSimZMQRobot.h"


/**
 * @brief _vstack stacks matrices in sequence vertically
 * @param A Top matrix
 * @param B Bottom matrix
 * @return The matrix [A; B] with same number of columns as A and B
 */
MatrixXd _vstack(const MatrixXd &A, const MatrixXd &B);

/**
 * @brief _vstack Vertically stacks two matrices with compatible column dimensions.
 * @param A Top matrix
 * @param B Bottom matrix
 * @return The vertically stacked matrix [A; B]
 * @throws std::runtime_error if column dimensions of A and B don't match
 */
MatrixXd _vstack(const MatrixXd &A, const MatrixXd &B)
{
    int m_A = A.rows();
    int m_B = B.rows();
    int n_A = A.cols();
    int n_B = B.cols();

    if (n_A != n_B)
        throw std::runtime_error(std::string("Wrong call of _vstack(A, B). ")
                                     + std::string("Incompatible sizes. The cols of Matrix A and B must have the same dimensions. ")
                                     + std::string("But A is ")+ std::to_string(A.rows())+ std::string("x")+ std::to_string(n_A)
                                     + std::string(" and B is ")+ std::to_string(B.rows()) + std::string("x")+ std::to_string(n_B));

    MatrixXd C = MatrixXd::Zero(m_A + m_B, n_A);
    C.block(0,0, m_A, n_A) = A;
    C.block(m_A, 0, m_B, n_B) = B;
    return C;
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::UnitreeB1Z1CoppeliaSimZMQRobot
 *        Constructor that initializes the CoppeliaSim interface for the
 *        Unitree B1 Z1 mobile manipulator.
 * @param robot_name Name of the robot in the CoppeliaSim scene
 * @param coppeliasim_interface_sptr Shared pointer to the ZMQ interface
 * @param model The robot model type (HOLONOMIC_MOBILE_MANIPULATOR or CFF_MANIPULATOR)
 */
UnitreeB1Z1CoppeliaSimZMQRobot::UnitreeB1Z1CoppeliaSimZMQRobot(const std::string &robot_name,
                                                               const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &coppeliasim_interface_sptr,
                                                               const MODEL &model)
    :DQ_CoppeliaSimRobotZMQ(robot_name, coppeliasim_interface_sptr), model_{model}
{
    _initialize_robot_objectnames_from_coppeliasim();
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::get_jointnames Returns the names of the
 *        arm joints (excluding the gripper).
 * @return Vector of joint name strings
 */
std::vector<std::string> UnitreeB1Z1CoppeliaSimZMQRobot::get_jointnames() const
{
    return jointnames_;
}


/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::_initialize_robot_objectnames_from_coppeliasim
 *        Initializes joint names and object names by querying the CoppeliaSim scene.
 *        Extracts arm joint names, separates the gripper joint, and sets the base name.
 */
void UnitreeB1Z1CoppeliaSimZMQRobot::_initialize_robot_objectnames_from_coppeliasim()
{
    alljointnames_ = _get_interface_sptr()->get_jointnames_from_object(robot_name_+"/UnitreeZ1");
    jointnames_ = alljointnames_;
    jointnames_.pop_back();
    gripper_jointname_ = alljointnames_.back();
    holonomic_base_name_ = robot_name_; //+"/trunk_respondable";
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::_get_mobile_robot_configuration_from_pose
 *        Extracts the mobile robot configuration (x, y, theta) from a dual quaternion pose.
 * @param base The base pose as a dual quaternion
 * @return Vector of 3 elements [x, y, theta] representing the planar configuration
 */
VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::_get_mobile_robot_configuration_from_pose(const DQ &base)
{
    auto x = base;
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    auto rangle = x.P().rotation_angle();
    return (VectorXd(3)<< p(0), p(1), rangle).finished();
}


/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::_get_joint_arm_positions
 *        Retrieves the current joint positions of the arm from CoppeliaSim.
 * @return Vector of arm joint positions
 */
VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::_get_joint_arm_positions()
{
    return _get_interface_sptr()->get_joint_positions(jointnames_);
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::_get_base_pose
 *        Retrieves the current base pose from CoppeliaSim.
 * @return The base pose as a dual quaternion
 */
DQ UnitreeB1Z1CoppeliaSimZMQRobot::_get_base_pose()
{
 return _get_interface_sptr()->get_object_pose(holonomic_base_name_);
}


/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration Sets the robot configuration
 *        in CoppeliaSim based on the specified model type.
 * @param q Configuration vector [vec8(base), arm_joints(6)]
 * @throws std::runtime_error If model type is unsupported or configuration is invalid
 */
void UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration(const VectorXd &q)
{
    switch (model_) {

        case MODEL::HOLONOMIC_MOBILE_MANIPULATOR:
            throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration:: Unsupported in the current MODEL");
        case MODEL::CFF_MANIPULATOR:
        {
            const VectorXd qbase = q.head(8);
            const VectorXd qarm  = q.tail(6);
            const DQ xbase = DQ(qbase);
            _get_interface_sptr()->set_object_pose(holonomic_base_name_, xbase);
            _get_interface_sptr()->set_joint_positions(jointnames_, qarm);
            break;
        }
        default:
            throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration:: Undefined model!");
    }

}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration Retrieves the current
 *        robot configuration from CoppeliaSim based on the model type.
 * @return Configuration vector [base_pose(3 or 8), arm_joints(6)]
 * @throws std::runtime_error If model type is undefined
 */
VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration()
{
    VectorXd qbase;
    switch (model_) {
        case MODEL::HOLONOMIC_MOBILE_MANIPULATOR:
            qbase = _get_mobile_robot_configuration_from_pose(_get_base_pose());
            break;
        case MODEL::CFF_MANIPULATOR:
            qbase = _get_base_pose().vec8();
            break;
        default:
            throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration():: Undefined model!");
    }
    VectorXd q = _vstack(qbase, _get_joint_arm_positions());
    return q;
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration Sets the target
 *        configuration for the robot (currently unsupported).
 * @param q_target Target configuration vector
 * @throws std::runtime_error Always throws as this method is not supported
 */
void UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration([[maybe_unused]] const VectorXd &q_target)
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration: Unsupported");
}


/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_velocities Retrieves
 *        the current configuration velocities (currently unsupported).
 * @throws std::runtime_error Always throws as this method is not supported
 * @return Empty vector (never reached due to exception)
 */
VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_velocities()
{
    throw std::runtime_error("UnitreeeB1Z1CoppeliaSimZMQRobot::get_configuration_velocities: Unsupported");
    return VectorXd::Zero(0);
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_velocities Sets
 *        the target configuration velocities (currently unsupported).
 * @param v_target Target velocity vector
 * @throws std::runtime_error Always throws as this method is not supported
 */
void UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_velocities([[maybe_unused]] const VectorXd &v_target)
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_space_velocities: Unsupported");
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_forces Sets
 *        the target configuration forces/torques (currently unsupported).
 * @param t Target force/torque vector
 * @throws std::runtime_error Always throws as this method is not supported
 */
void UnitreeB1Z1CoppeliaSimZMQRobot::set_target_configuration_forces([[maybe_unused]] const VectorXd &t)
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::set_configuration_space_torques: Unsupported");
}

/**
 * @brief UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_forces Retrieves the
 *        current configuration forces/torques (currently unsupported).
 * @throws std::runtime_error Always throws as this method is not supported
 * @return Empty vector (never reached due to exception)
 */
VectorXd UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_forces()
{
    throw std::runtime_error("UnitreeB1Z1CoppeliaSimZMQRobot::get_configuration_space_torques: Unsupported");
    return VectorXd::Zero(0);
}




