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
#    Contributors:
#
#    1. Bruno Vilhena Adorno (bruno.adorno@manchester.ac.uk)
#       Responsible for the original implementation in file DQ_FreeFlyingRobot.m
#       https://github.com/dqrobotics/matlab/pull/66/commits
#
#    2. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
#       Created the first version of this class for C++, from which this
#       version was adapted.
#
#    3. Daniel S. J. Derwent (daniel.derwent@manchester.ac.uk)
#       Adapted Juan's C++ class into the version below.
#
# ################################################################
*/

/*
    The configurations provided to the DQ_FreeFlyingRobot are assumed to be of the form

        q = vec8(x)

    where x is the base pose. However, it is also assumed that any control applications
    making use of DQ_FreeFlyingRobot will use

        u = vec8(twist)

    rather than u = vec8(x_dot). As a result, the function fkm() takes a pose and returns
    that same pose, while the function pose_jacobian() takes a pose and returns the 
    matrix J such that x_dot = J * twist. The function pose_jacobian_derivative()
    takes a pose derivative x_dot and returns the time derivative of J above.
    
*/

#include <cmath>
#include <dqrobotics/robot_modeling/DQ_FreeFlyingRobot.h>

namespace DQ_robotics
{

    /**
     * @brief class constructor.
     */
    DQ_FreeFlyingRobot::DQ_FreeFlyingRobot()
    {
        // The configuration space really has 6 DOF because its a pose, but
        // the configuration vectors passed to the class contain 8 elements.
        dim_configuration_space_ = 8;
    }

    /**
     * @brief convenience function to get the size of the "configuration" vector
     * @returns the size of the configuration vector
     */
    int DQ_FreeFlyingRobot::get_len_config_vector() const
    {
        // The configuration vector is presumed to be vec8(pose)
        return (dim_configuration_space_);
    }

    /**
     * @brief convenience function to get the size of the control input vector
     * @returns the size of the control input vector
     */
    int DQ_FreeFlyingRobot::get_len_control_input_vector() const
    {
        // The control input is presumed to be a vec8(twist)
        return (dim_configuration_space_);
    }

    /**
     * @brief convenience function which verifies that the configuration vector
     * q is a pose.
     * @param q The configuration vector being tested.
     */
    void DQ_FreeFlyingRobot::_check_pose(const VectorXd &q) const
    {
        if(q.size() != get_len_config_vector()){
            throw std::runtime_error("[DQ_FreeFlyingRobot::_check_pose] Passed configuration vector of incorrect length!");
        }

        if (DQ(q).norm() != 1) {
            throw std::runtime_error("[DQ_FreeFlyingRobot::_check_pose] Passed configuration vector that was not a unit norm dual quaternion!");
        }
    }

    /**
     * @brief convenience function which verifies that the pose derivative is the right length
     * @param twist The vector being tested.
     */
    void DQ_FreeFlyingRobot::_check_pose_derivative(const VectorXd &q_dot) const
    {
        if(q_dot.size() != get_len_config_vector()){
            throw std::runtime_error("[DQ_FreeFlyingRobot::_check_pose_derivative] Passed pose derivative vector of incorrect length!");
        }
    }

    /**
     * @brief returns the pose of the base given the current pose. This function only exists
     * to ensure compatibility with other DQ Robotics classes.
     * @param q The VectorXd representing the robot pose.
     */
    DQ DQ_FreeFlyingRobot::fkm(const VectorXd &q) const
    {
        _check_pose(q);

        return DQ(q);
    }

    /**
     * @brief returns the pose of the base given the current pose. This function only exists
     * to ensure compatibility with other DQ Robotics classes.
     * @param q The VectorXd representing the robot pose.
     * @param to_link This input is ignored, included only for compatibility reasons.
     */
    DQ DQ_FreeFlyingRobot::fkm(const VectorXd &q, const int &/*to_link*/) const
    {
        // Ignore the to_link input since a FreeFlyingRobot doesn't have discrete
        // links to consider individually.
        return fkm(q);
    }

    /**
     * @brief returns the Jacobian matrix J that satisfies
     * 
     *        vec8(x_dot) = J*vec8(twist),
     * 
     *        where x_dot is the time derivative of the unit dual quaternion x
     *        that represents the free-flying robot pose, and twist satisfies 
     *        the dual quaternion propagation equation
     * 
     *        xdot = (1/2)*twist*x.
     * 
     * @param q The VectorXd representing the robot pose.
     */
    MatrixXd DQ_FreeFlyingRobot::pose_jacobian(const VectorXd &q) const
    {
        _check_pose(q);
        DQ pose(q);

        return haminus8(0.5*pose);
    }

    /**
     * @brief returns the Jacobian matrix J that satisfies
     * 
     *        vec8(x_dot) = J*vec8(twist),
     * 
     *        where x_dot is the time derivative of the unit dual quaternion x
     *        that represents the free-flying robot pose, and twist satisfies 
     *        the dual quaternion propagation equation
     * 
     *        x_dot = (1/2)*twist*x.
     * 
     * @param q The VectorXd representing the robot pose.
     * @param to_link This input is ignored, included only for compatibility reasons.
     */
    MatrixXd DQ_FreeFlyingRobot::pose_jacobian(const VectorXd &q, const int &/*to_link*/) const
    {
        // Ignore the to_link input since a FreeFlyingRobot doesn't have discrete
        // links to consider individually.
        return pose_jacobian(q);
    }

    /**
     * @brief This method returns the pose Jacobian derivative of J, where
     *        J is the pose Jacobian that satisfies
     *
     *        vec8(x_dot) = J*vec8(twist),
     *
     *        where x_dot is the time derivative of the unit dual quaternion x
     *        that represents the free-flying robot pose, and twist satisfies 
     *        the dual quaternion propagation equation
     * 
     *        x_dot = (1/2)*twist*x.
     * 
     * @param q This input is ignored, included only for compatibility reasons.
     * @param q_dot The robot pose derivative vec8(x_dot)
     * @return a MatrixXd representing the desired Jacobian derivative.
     */
    MatrixXd DQ_FreeFlyingRobot::pose_jacobian_derivative(const VectorXd &/*q*/, const VectorXd &q_dot) const
    {
        _check_pose_derivative(q_dot);
        
        DQ pose_derivative(q_dot);
        return haminus8(0.5*pose_derivative);
    }

    /**
     * @brief This method returns the pose Jacobian derivative of J, where
     *        J is the pose Jacobian that satisfies
     *
     *        vec8(x_dot) = J*vec8(twist),
     *
     *        where x_dot is the time derivative of the unit dual quaternion x
     *        that represents the free-flying robot pose, and twist satisfies 
     *        the dual quaternion propagation equation
     * 
     *        x_dot = (1/2)*twist*x.
     * 
     * @param q This input is ignored, included only for compatibility reasons.
     * @param q_dot The robot pose derivative vec8(x_dot)
     * @param to_link This input is ignored, included only for compatibility reasons.
     * @return a MatrixXd representing the desired Jacobian derivative.
     */
    MatrixXd DQ_FreeFlyingRobot::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &/*to_link*/) const
    {
        return pose_jacobian_derivative(q, q_dot);
    }

}