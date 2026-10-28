/*
#    Copyright (c) 2026 Adorno-Lab
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
# ################################################################
*/

/*
    The configurations provided to the DQ_FreeFloatingBase are assumed to be of the form

        q = vec8(x)

    where x is the base pose. This requirement renders a lot of the kinematics functions
    trivial (e.g., the Jacobian matrix is simply the identity matrix). However, this class
    is useful to serve as a base on which to attach other kinematic bodies.
*/

#include <dqrobotics/robot_modeling/DQ_FreeFloatingBase.h>

namespace DQ_robotics
{

    /**
     * @brief class constructor.
     */
    DQ_FreeFloatingBase::DQ_FreeFloatingBase()
    {
        // The configuration space has 6 DOF because its a pose, but
        // the configuration vectors passed to the class contain 8 elements
        dim_configuration_space_ = 6;
    }

    /**
     * @brief convenience function which verifies that the configuration vector
     * q has exactly 8 elements and is of unit norm.
     * @param q The configuration vector being tested.
     */
    void DQ_FreeFloatingBase::_check_q(const VectorXd &q) const
    {
        if(q.size() != 8){
            throw std::runtime_error("[DQ_FreeFloatingBase::_check_q] Passed configuration vector of incorrect length! Must be 8 elements long.");
        }

        if (DQ(q).norm() != 1) {
            throw std::runtime_error("[DQ_FreeFloatingBase::_check_q] Passed configuration vector that was not a unit norm dual quaternion!");
        }
    }

    /**
     * @brief convenience function which verifies that the configuration velocity
     * vector q_dot has exactly 8 elements.
     * @param q_dot The configuration velocity vector being tested.
     */
    void DQ_FreeFloatingBase::_check_q_dot(const VectorXd &q_dot) const
    {
        if(q_dot.size() != 8){
            throw std::runtime_error("[DQ_FreeFloatingBase::_check_q_dot] Passed configuration velocity vector of incorrect length! Must be 8 elements long.");
        }
    }

    /**
     * @brief convenience function which verifies that the link index provide is
     * equal to 5. This is necessary for this class since the free floating base does 
     * not contain discrete links that can be isolated.
     * @param link The link index being tested.
     */
    void DQ_FreeFloatingBase::_check_link(const int &link) const
    {
        if(link != dim_configuration_space_ - 1){
            throw std::runtime_error("[DQ_FreeFloatingBase::_check_link] Passed an invalid link! DQ_FreeFloatingBase only accepts link = 5.");
        }
    } 

    /**
     * @brief returns the pose of the base given the current configuration.
     * @param q The VectorXd representing the robot configuration.
     */
    DQ DQ_FreeFloatingBase::fkm(const VectorXd &q) const
    {
        _check_q(q);

        // q is already a pose
        return DQ(q) * frame_displacement_;
    }

    /**
     * @brief returns the pose of the base given the current configuration.
     * @param q The VectorXd representing the robot configuration.
     * @param to_link The link for which we want to compute the pose. 
     *        for DQ_FreeFloatingBase this value must be 5.
     */
    DQ DQ_FreeFloatingBase::fkm(const VectorXd &q, const int &to_link) const
    {
        _check_q(q);
        _check_link(to_link);
        return fkm(q);
    }

    /**
     * @brief returns the Jacobian matrix J that satisfies
              vec8(pose_dot) = J * q_dot, where 'pose_dot' is the time
              derivative of the pose and 'q_dot' represents the configuration velocities.
     * @param q The VectorXd representing the robot configuration.
     * @param to_link The link for which we want to compute the Jacobian.
     *        for DQ_FreeFloatingBase this value must be 5.
     */
    MatrixXd DQ_FreeFloatingBase::pose_jacobian(const VectorXd &q, const int &to_link) const
    {
        _check_q(q);
        _check_link(to_link);

        // Since q = vec*(pose), the pose Jacobian is just the identity matrix.
        return pose_jacobian(q);
    }

    /**
     * @brief returns the Jacobian matrix J that satisfies
              vec8(pose_dot) = J * q_dot, where 'pose_dot' is the time
              derivative of the pose and 'q_dot' represents the configuration velocities.
     * @param q The VectorXd representing the robot configuration.
     */
    MatrixXd DQ_FreeFloatingBase::pose_jacobian(const VectorXd &q) const
    {
        _check_q(q);

        return haminus8(frame_displacement_) * MatrixXd::Identity(8,8);
    }

    /**
     * @brief returns the Jacobian derivative 'J_dot' that satisfies
              vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
              derivative of the pose and 'q_dot' represents the robot configuration velocities.
     * @param q The VectorXd representing the robot configurations.
     * @param q_dot The VectorXd representing the robot configuration velocities.
     * @param to_link The link for which we want to compute the Jacobian derivative.
     *        for DQ_FreeFloatingBase this value must be 5.
     * @return a MatrixXd representing the desired Jacobian derivative.
     */
    MatrixXd DQ_FreeFloatingBase::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_link) const
    {
        _check_q(q);
        _check_q_dot(q_dot);
        _check_link(to_link);

        return pose_jacobian_derivative(q, q_dot);
    }

    /**
     * @brief returns the Jacobian derivative 'J_dot' that satisfies
              vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
              derivative of the pose and 'q_dot' represents the robot configuration velocities.
     * @param q The VectorXd representing the robot configurations.
     * @param q_dot The VectorXd representing the robot configuration velocities.
     * @return a MatrixXd representing the desired Jacobian derivative.
     */
    MatrixXd DQ_FreeFloatingBase::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
    {
        _check_q(q);
        _check_q_dot(q_dot);

        // Since J = I, the time derivative of J is 0.
        return MatrixXd::Zero(8,8);
    }

}