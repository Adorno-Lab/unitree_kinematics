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

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLOATINGBASE
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_FREEFLOATINGBASE

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_MobileBase.h>

namespace DQ_robotics
{


class DQ_FreeFloatingBase: public DQ_MobileBase
{
public:
    // Constructor
    DQ_FreeFloatingBase();

    // Virtual method overrides
    virtual DQ fkm(const VectorXd& q) const override;
    virtual DQ fkm(const VectorXd& q, const int& to_ith_link) const override;
    virtual MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const override;
    virtual MatrixXd pose_jacobian(const VectorXd& q) const override;
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_link) const override;
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot) const override;

private:

    // Private methods for input validation
    void _check_q(const VectorXd& q) const;
    void _check_q_dot(const VectorXd& q_dot) const;
    void _check_link(const int& link) const;
};

}

#endif