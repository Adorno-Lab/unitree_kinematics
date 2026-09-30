/*
 * Smoke test for the installed unitree_kinematics package.
 *
 * Builds each kinematic model and checks its configuration-space size, that
 * fkm() returns a unit dual quaternion, and that pose_jacobian() has the
 * expected shape. The CoppeliaSim class is only compiled and linked, not run,
 * since that would need a running simulator.
 */
#include <dqrobotics/robots/UnitreeZ1Robot.h>
#include <dqrobotics/robots/UnitreeB1Z1MobileRobot.h>
#include <dqrobotics/robots/CFFSerialRobot.h>
#include <dqrobotics/interfaces/coppeliasim/robots/UnitreeB1Z1CoppeliaSimZMQRobot.h>

#include <iostream>
#include <memory>
#include <string>

using namespace DQ_robotics;

namespace
{

int failures = 0;

void check(bool condition, const std::string& what)
{
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << what << std::endl;
    if (!condition)
        ++failures;
}

void check_model(const std::string& name, const DQ_Kinematics& robot,
                 const VectorXd& q, int expected_dim)
{
    const int n = robot.get_dim_configuration_space();
    check(n == expected_dim,
          name + " has " + std::to_string(expected_dim) + " DoF (got " + std::to_string(n) + ")");

    check(is_unit(robot.fkm(q)), name + " fkm(q) is a unit dual quaternion");

    const MatrixXd J = robot.pose_jacobian(q);
    check(J.rows() == 8 && J.cols() == n, name + " pose_jacobian(q) is 8x" + std::to_string(n));
}

} // namespace

// Compile/link check only -- never called, since it would need a running
// CoppeliaSim. External linkage forces the compiler to emit it, so the linker
// must resolve the class's constructor, vtable, and methods from the installed
// libunitree_kinematics.
void coppeliasim_compile_and_link_check();
void coppeliasim_compile_and_link_check()
{
    auto interface = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    for (const auto model : {UnitreeB1Z1CoppeliaSimZMQRobot::MODEL::HOLONOMIC_MOBILE_MANIPULATOR,
                             UnitreeB1Z1CoppeliaSimZMQRobot::MODEL::CFF_MANIPULATOR})
    {
        UnitreeB1Z1CoppeliaSimZMQRobot robot("UnitreeB1_1", interface, model);
        static_cast<void>(robot.get_jointnames());
        const VectorXd q = robot.get_configuration();
        robot.set_configuration(q);
        robot.set_target_configuration(q);
        robot.set_target_configuration_velocities(robot.get_configuration_velocities());
        robot.set_target_configuration_forces(robot.get_configuration_forces());
    }
}

int main()
{
    const VectorXd q_arm = VectorXd::LinSpaced(6, 0.1, 0.5);

    const auto z1 = UnitreeZ1Robot::kinematics();
    check_model("UnitreeZ1Robot", z1, q_arm, 6);

    // q = [x, y, phi, arm joints]. Both offsets must be set before use; they
    // are not initialized by the constructor.
    UnitreeB1Z1MobileRobot b1z1;
    b1z1.update_base_offset(1 + 0.5*E_*(0.1*i_ + 0.2*k_));
    b1z1.update_base_height_from_IMU(1 + 0.5*E_*(0.4*k_));
    VectorXd q_b1z1(9);
    q_b1z1 << 0.3, -0.2, 0.5, q_arm;
    check_model("UnitreeB1Z1MobileRobot", b1z1, q_b1z1, 9);

    // q = [vec8(base pose), arm joints] (14 entries), while the configuration
    // space (and the Jacobian's columns) is 6 base twist DoF + 6 arm DoF.
    const CFFSerialRobot cff(std::make_shared<DQ_SerialManipulatorDH>(z1));
    const DQ x_base = (cos(0.25) + k_*sin(0.25)) * (1 + 0.5*E_*(0.3*i_ - 0.2*j_ + 0.4*k_));
    VectorXd q_cff(14);
    q_cff << vec8(x_base), q_arm;
    check_model("CFFSerialRobot", cff, q_cff, 12);

    // Reaching this point means coppeliasim_compile_and_link_check() compiled
    // and linked; it is deliberately not run.
    std::cout << "[PASS] UnitreeB1Z1CoppeliaSimZMQRobot compiles and links (not run)" << std::endl;

    if (failures > 0)
    {
        std::cerr << failures << " check(s) failed" << std::endl;
        return 1;
    }
    std::cout << "All checks passed" << std::endl;
    return 0;
}
