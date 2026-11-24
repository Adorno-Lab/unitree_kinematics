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

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLYINGROBOT
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLYINGROBOT

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_MobileBase.h>

namespace DQ_robotics
{


class DQ_FreeFlyingRobot: public DQ_MobileBase
{
public:
    // Constructor
    DQ_FreeFlyingRobot();

    // Virtual method overrides
    virtual DQ fkm(const VectorXd& q) const override;
    virtual DQ fkm(const VectorXd& q, const int& to_ith_link) const override;
    virtual MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const override;
    virtual MatrixXd pose_jacobian(const VectorXd& q) const override;
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_link) const override;
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot) const override;

    // New public methods
    int get_len_config_vector() const;
    int get_len_control_input_vector() const;

private:

    // Private methods for input validation
    void _check_pose(const VectorXd& q) const;
    void _check_pose_derivative(const VectorXd& q_dot) const;
};

}

#endif