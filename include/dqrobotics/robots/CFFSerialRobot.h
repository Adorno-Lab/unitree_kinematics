#pragma once
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <memory>

namespace DQ_robotics
{
class CFFSerialRobot: public DQ_Kinematics
{
public:
    enum class SIX_DOF_CONSTRAINT_MODE{NONE, PLANAR_JOINT, STAND};
private:
    DQ x_b_a_{1};
    std::shared_ptr<DQ_SerialManipulator> kin_arm_;
    const int base_dim_configuration_space_ = 6;
    const int arm_dim_configuration_space_;
    MatrixXd I_ = MatrixXd::Zero(8,6);
    int min_index_;
    int max_index_;
public:
    CFFSerialRobot(const std::shared_ptr<DQ_SerialManipulator>& robot_arm);
    CFFSerialRobot() = delete;

    void set_offset(const DQ& x_b_a);
    std::tuple<MatrixXd, VectorXd> get_six_dof_constraints(const SIX_DOF_CONSTRAINT_MODE& mode);

    //Abstract methods' implementation
    DQ fkm(const VectorXd& q, const int& ith) const override;
    DQ fkm(const VectorXd& q) const override;
    MatrixXd pose_jacobian(const VectorXd& q, const int& ith) const override;
    MatrixXd pose_jacobian(const VectorXd& q) const override;

    int get_dim_configuration_space() const override;

    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_ith_link) const override; //To be implemented.
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                      const VectorXd& q_dot) const override; //To be implemented.

};
}


