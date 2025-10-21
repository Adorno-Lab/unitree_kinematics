#include "HolonomicB1Z1CoppeliaSim.h"
#include <dqrobotics_extensions/robot_constraint_manager/utils.hpp>

namespace DQ_robotics
{
HolonomicB1Z1CoppeliaSim::HolonomicB1Z1CoppeliaSim(const std::string &robot_name,
                                                   const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &coppeliasim_interface_sptr)
  :DQ_CoppeliaSimRobotZMQ(robot_name, coppeliasim_interface_sptr), robot_name_{robot_name}
{
    arm_jointnames_ = _get_interface_sptr()->get_jointnames_from_object(robot_name_+"/UnitreeZ1");
    arm_jointnames_.pop_back();

    base_jointnames_ = {robot_name_+"/rollingJoint_fl", robot_name_+"/rollingJoint_rl",
                        robot_name_+"/rollingJoint_rr", robot_name_+"/rollingJoint_fr"};

    robot_model_ = std::make_shared<UnitreeB1Z1MobileRobot>();


    DQ X_J1  = _get_interface_sptr()->get_object_pose(arm_jointnames_.at(0));
    DQ X_IMU = _get_interface_sptr()->get_object_pose(robot_name_+"/IMU");
    DQ X_J1_OFFSET = X_IMU.conj()*X_J1;
    robot_model_ ->update_base_offset(X_J1_OFFSET);
    robot_model_ ->update_base_height_from_IMU(X_IMU);






}

void HolonomicB1Z1CoppeliaSim::set_configuration([[maybe_unused]] const VectorXd &configuration)
{
    throw std::runtime_error(std::string(__FUNCTION__)+"::Not implemented!");
}

void HolonomicB1Z1CoppeliaSim::set_target_configuration([[maybe_unused]] const VectorXd &target_configuration)
{
    throw std::runtime_error(std::string(__FUNCTION__)+"::Not implemented!");
}

void HolonomicB1Z1CoppeliaSim::set_target_configuration_velocities(const VectorXd &target_configuration_velocities)
{
    set_target_holonomic_velocities(target_configuration_velocities.head(3));
    _get_interface_sptr()->set_joint_target_velocities(arm_jointnames_, target_configuration_velocities.tail(6));
}

void HolonomicB1Z1CoppeliaSim::set_target_configuration_forces([[maybe_unused]] const VectorXd &target_configuration_forces)
{
    throw std::runtime_error(std::string(__FUNCTION__)+"::Not implemented!");
}

VectorXd HolonomicB1Z1CoppeliaSim::get_configuration()
{
    VectorXd q = VectorXd::Zero(9);
    VectorXd qarm = _get_interface_sptr()->get_joint_positions(arm_jointnames_);
    DQ xbase = _get_interface_sptr()->get_object_pose(robot_name_+"/IMU");
    VectorXd qbase = _get_mobile_robot_configuration_from_pose(xbase);
    q << qbase, qarm;
    return q;
}

VectorXd HolonomicB1Z1CoppeliaSim::get_configuration_velocities()
{
    throw std::runtime_error(std::string(__FUNCTION__)+"::Not implemented!");
}

VectorXd HolonomicB1Z1CoppeliaSim::get_configuration_forces()
{
    throw std::runtime_error(std::string(__FUNCTION__)+"::Not implemented!");
}

void HolonomicB1Z1CoppeliaSim::set_target_holonomic_velocities(const VectorXd &u_base)
{
    const double& forwBackVel = u_base(0);
    const double& leftRightVel = u_base(1);
    const double& rotVel = u_base(2);

    VectorXd vel = VectorXd::Zero(4);
    vel << -forwBackVel+leftRightVel+rotVel, -forwBackVel-leftRightVel+rotVel,
           -forwBackVel+leftRightVel-rotVel, -forwBackVel-leftRightVel-rotVel;

    _get_interface_sptr()->set_joint_target_velocities(base_jointnames_, vel);
}

void HolonomicB1Z1CoppeliaSim::set_arm_joint_target_positions(const VectorXd &q_arm)
{
    _get_interface_sptr()->set_joint_target_positions(arm_jointnames_, q_arm);
}

VectorXd HolonomicB1Z1CoppeliaSim::get_planar_joint_velocities_at_body_frame(const VectorXd &qbase, const VectorXd &planar_joint_velocities_at_inertial_frame) const
{
    const double phi = qbase(2);
    const DQ rbase = cos(phi/2) + k_*sin(phi/2);
    const DQ pbase = qbase(0)*i_ + qbase(1)*j_;

    const DQ robot_pose = rbase + E_*0.5*pbase*rbase;
    const VectorXd& ua = planar_joint_velocities_at_inertial_frame; // x_dot, y_dot, phi_dot
    DQ p_dot_a_ab = ua(0)*i_ + ua(1)*j_;
    DQ w_a_ab = ua(2)*k_;
    DQ twist_a = w_a_ab + E_*(p_dot_a_ab + DQ_robotics::cross(robot_pose.translation(), w_a_ab));
    // Twist_a expressed in the body frame is given as
    DQ twist_b = Ad(robot_pose.conj(), twist_a);
    VectorXd twist_b_vec = twist_b.vec6(); // [0 0 wb xb_dot yb_dot 0]
        //[xb_dot         yb_dot        wb]
    return DQ_robotics_extensions::CVectorXd({twist_b_vec(3), twist_b_vec(4), twist_b_vec(2)});
}

std::shared_ptr<DQ_Kinematics> HolonomicB1Z1CoppeliaSim::get_model() const
{
    return std::shared_ptr<DQ_Kinematics>(robot_model_);
}


VectorXd HolonomicB1Z1CoppeliaSim::_get_mobile_robot_configuration_from_pose(const DQ &base) const
{
    auto x = base;
    auto axis = x.rotation_axis().vec4();
    if (axis(3)<0)
        x = -x;
    auto p = x.translation().vec3();
    double angle = x.P().rotation_angle();

    //while (angle >= 2.0*M_PI) angle -= 2.0 * M_PI;
    //while (angle < 0) angle += 2.0 * M_PI;

    return (VectorXd(3)<< p(0), p(1), angle).finished();
}




}
