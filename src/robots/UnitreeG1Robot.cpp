/*
#    Copyright (c) 2024-2026 Adorno-Lab
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
#   Author: - Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#           - This code was reviewed and documented by Claude.
#
# ################################################################
*/

#include<cmath>
#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/robots/UnitreeG1Robot.h>

namespace DQ_robotics
{

DQ_SerialManipulatorDH _get_g1_waist_chain();
DQ_SerialManipulatorDH _get_g1_left_leg_chain();
DQ_SerialManipulatorDH _get_g1_right_leg_chain();
DQ_SerialManipulatorDH _get_g1_right_arm_chain();
DQ_SerialManipulatorDH _get_g1_left_arm_chain();



DQ_SerialManipulatorDH UnitreeG1Robot::kinematics(const LIMB& limb)
{
    switch (limb) {

    case LIMB::LEFT_ARM:
        return _get_g1_left_arm_chain();
    case LIMB::RIGHT_ARM:
        return _get_g1_right_arm_chain();
    case LIMB::WAIST:
        return _get_g1_waist_chain();
    case LIMB::LEFT_LEG:
        return _get_g1_left_leg_chain();
    case LIMB::RIGHT_LEG:
        return _get_g1_right_leg_chain();
    default:
        throw std::runtime_error("Invalid limb");
    }
}


DQ_SerialManipulatorDH _get_g1_left_leg_chain()
{
    const double pi2 = M_PI/2;

    // theta,   d,        a,       alpha,  type   (standard DH)
    Matrix<double,5,6> g1_left_leg_dh(5,6);
    g1_left_leg_dh <<
        1.39589633, -pi2,        -pi2,        -1.39589633,  0,    0,
        0.05200000,  0.01969980, -0.30146000,  0.00214890,  0,    0,
        0.03000022,  0,           0.07827300,  0.30001000,  0.017558, 0,
        pi2,         pi2,         pi2,          0,          pi2,  0,
        0,           0,           0,            0,          0,    0;

    DQ_SerialManipulatorDH g1_left_leg(g1_left_leg_dh);

    // Base transform: pelvis -> DH frame 0 (rotation + translation)
    // A single elementary rotation of -pi/2 about x (frame 0's x-axis
    // coincides with the pelvis x-axis; z is tilted -90 deg to align with
    // the hip-pitch joint axis). Verified directly against the DH
    // derivation's own base-rotation matrix (residual ~8.7e-17).
    const double phi_base = -pi2;    // rotation angle about x
    const DQ r_base = cos(phi_base/2) + i_*sin(phi_base/2);              // Rx(phi_base)
    const DQ t_base = 0*i_ + 0.06445200*j_ - 0.10270000*k_;              // pure quaternion translation
    const DQ x_base = r_base + E_*0.5*t_base*r_base;
    g1_left_leg.set_reference_frame(x_base);
    g1_left_leg.set_base_frame(x_base);

    // Effector transform: DH frame 6 -> left_ankle_roll_link (pure rotation,
    // zero translation). A single elementary rotation of -pi/2 about y.
    // Verified directly against the DH derivation's own effector-rotation
    // matrix (residual ~3.3e-16).
    const double phi_effector = -pi2;    // rotation angle about y
    const DQ r_effector = cos(phi_effector/2) + j_*sin(phi_effector/2);  // Ry(phi_effector)
    const DQ t_effector = 0*i_ + 0*j_ + 0*k_;
    const DQ x_effector = r_effector + E_*0.5*t_effector*r_effector;
    g1_left_leg.set_effector(x_effector);

    return g1_left_leg;
}

DQ_SerialManipulatorDH _get_g1_waist_chain()
{
    const double pi2 = M_PI/2;

    // theta,   d,           a,          alpha,  type   (standard DH)
    Matrix<double,5,3> g1_waist_dh(5,3);
    g1_waist_dh <<
        pi2,          pi2,          0,
        0.03500000,  -0.00396350,   0,
        0,             0.01900000,  0,
        pi2,           pi2,         0,
        0,             0,           0;

    DQ_SerialManipulatorDH g1_waist(g1_waist_dh);

    // Base transform: pelvis -> DH frame 0 is the identity here
    // (waist_yaw's axis already coincides with the pelvis z-axis).

    // Effector transform: pure rotation, composed of
    // two elementary rotations:
    //   r1: rotation of phi1 = pi/2 about z
    //   r2: rotation of phi2 = pi/2 about x
    const double phi1 = pi2;   // rotation angle about z
    const double phi2 = pi2;   // rotation angle about x
    const DQ r1 = cos(phi1/2) + k_*sin(phi1/2);   // Rz(phi1)
    const DQ r2 = cos(phi2/2) + i_*sin(phi2/2);   // Rx(phi2)
    const DQ r_effector = r1*r2;
    const DQ t_effector = 0*i_ + 0*j_ + 0*k_;
    const DQ x_effector = r_effector + E_*0.5*t_effector*r_effector;
    g1_waist.set_effector(x_effector);

    return g1_waist;
}

DQ_SerialManipulatorDH _get_g1_right_leg_chain()
{
    const double pi2 = M_PI/2;

    // theta,   d,        a,       alpha,  type   (standard DH)
    Matrix<double,5,6> g1_right_leg_dh(5,6);
    g1_right_leg_dh <<
        1.39589633, -pi2,        -pi2,        -1.39589633,  0,    0,
        -0.05200000,  0.01969980, -0.30146000, -0.00214890,  0,    0,
        0.03000022,  0,           0.07827300,  0.30001000,  0.017558, 0,
        pi2,         pi2,         pi2,          0,          pi2,  0,
        0,           0,           0,            0,          0,    0;

    DQ_SerialManipulatorDH g1_right_leg(g1_right_leg_dh);

    // Base transform: pelvis -> DH frame 0
    // Same elementary rotation as G1LeftLegRobot (see comment there):
    // Rx(-pi/2). Only the translation differs (mirrored in y).
    const double phi_base = -pi2;    // rotation angle about x
    const DQ r_base = cos(phi_base/2) + i_*sin(phi_base/2);              // Rx(phi_base)
    const DQ t_base = 0*i_ - 0.06445200*j_ - 0.10270000*k_;
    const DQ x_base = r_base + E_*0.5*t_base*r_base;
    g1_right_leg.set_reference_frame(x_base);
    g1_right_leg.set_base_frame(x_base);

    // Effector transform: DH frame 6 -> right_ankle_roll_link (pure
    // rotation, zero translation). Same rotation as the left leg (see
    // comment there): Ry(-pi/2).
    const double phi_effector = -pi2;    // rotation angle about y
    const DQ r_effector = cos(phi_effector/2) + j_*sin(phi_effector/2);  // Ry(phi_effector)
    const DQ t_effector = 0*i_ + 0*j_ + 0*k_;
    const DQ x_effector = r_effector + E_*0.5*t_effector*r_effector;
    g1_right_leg.set_effector(x_effector);

    return g1_right_leg;
}

DQ_SerialManipulatorDH _get_g1_right_arm_chain()
{
    const double pi2 = M_PI/2;

    // theta,   d,          a,           alpha,  type   (standard DH)
    Matrix<double,5,7> g1_right_arm_dh(5,7);
    g1_right_arm_dh <<
        pi2,        -1.29154633, -pi2,        -pi2,         M_PI, pi2,  0,
        -0.03800000,  0,          -0.18371800, -0.00188791,  0.13800000, 0, 0,
        0.01383100,  0.00624000, -0.01578300,  0.01000000, -0.00000000, 0.04600000, 0,
        pi2,         pi2,         pi2,          pi2,         pi2,  pi2, 0,
        0,           0,           0,            0,           0,    0,   0;

    DQ_SerialManipulatorDH g1_right_arm(g1_right_arm_dh);

    // Base transform: torso_link -> DH frame 0
    // Mirror of G1LeftArmRobot's base transform (see the comment there):
    // right_shoulder_pitch_joint's URDF origin rpy gives (roll, pitch, yaw)
    // below, and the same -pi/2 fixed rotation about x (needed because the
    // joint's axis is "0 1 0", not z) folds additively into the roll term.
    // Verified numerically against the URDF (residual ~5.9e-10).
    const double roll  = -0.27931;
    const double pitch =  5.4949e-05;
    const double yaw   =  0.00019159;
    const DQ r1 = cos(yaw/2)            + k_*sin(yaw/2);            // Rz(yaw)
    const DQ r2 = cos(pitch/2)          + j_*sin(pitch/2);          // Ry(pitch)
    const DQ r3 = cos((roll - pi2)/2)   + i_*sin((roll - pi2)/2);   // Rx(roll - pi/2)
    const DQ r_base = r1*r2*r3;
    const DQ t_base = 0.00395630*i_ - 0.10021000*j_ + 0.23778000*k_;
    const DQ x_base = r_base + E_*0.5*t_base*r_base;
    g1_right_arm.set_reference_frame(x_base);
    g1_right_arm.set_base_frame(x_base);

    // Effector transform: DH frame 7 -> right_rubber_hand (palm), pure translation
    const DQ r_effector = DQ(1);
    const DQ t_effector = 0.04150000*i_ - 0.00300000*j_ + 0*k_;
    const DQ x_effector = r_effector + E_*0.5*t_effector*r_effector;
    g1_right_arm.set_effector(x_effector);

    return g1_right_arm;
}

DQ_SerialManipulatorDH _get_g1_left_arm_chain()
{
    const double pi2 = M_PI/2;

    // theta,   d,          a,           alpha,  type   (standard DH)
    Matrix<double,5,7> g1_left_arm_dh(5,7);
    g1_left_arm_dh <<
        pi2,        -1.85004633, -pi2,        -pi2,         M_PI, pi2,  0,
        0.03800000,  0,          -0.18371800,  0.00188791,  0.13800000, 0, 0,
        0.01383100, -0.00624000, -0.01578300,  0.01000000, -0.00000000, 0.04600000, 0,
        pi2,         pi2,         pi2,          pi2,         pi2,  pi2, 0,
        0,           0,           0,            0,           0,    0,   0;

    DQ_SerialManipulatorDH g1_left_arm(g1_left_arm_dh);

    // Base transform: torso_link -> DH frame 0
    // The left_shoulder_pitch_joint frame (URDF origin rpy = roll, pitch, yaw
    // below) does not rotate about its own z-axis (its joint axis is "0 1 0"),
    // so an extra fixed rotation of -pi/2 about x is folded in to align the
    // frame with the standard-DH convention (which always rotates about z).
    // Since that extra term shares the x-axis with the roll term, they combine
    // additively into (roll - pi/2). Verified numerically against the URDF
    // (rotation-matrix residual ~5.9e-10, i.e. floating-point-exact).
    const double roll  =  0.27931;
    const double pitch =  5.4949e-05;
    const double yaw   = -0.00019159;
    const DQ r1 = cos(yaw/2)            + k_*sin(yaw/2);            // Rz(yaw)
    const DQ r2 = cos(pitch/2)          + j_*sin(pitch/2);          // Ry(pitch)
    const DQ r3 = cos((roll - pi2)/2)   + i_*sin((roll - pi2)/2);   // Rx(roll - pi/2)
    const DQ r_base = r1*r2*r3;
    const DQ t_base = 0.00395630*i_ + 0.10022000*j_ + 0.23778000*k_;
    const DQ x_base = r_base + E_*0.5*t_base*r_base;
    g1_left_arm.set_reference_frame(x_base);
    g1_left_arm.set_base_frame(x_base);

    // Effector transform: DH frame 7 -> left_rubber_hand (palm), pure translation
    const DQ r_effector = DQ(1);
    const DQ t_effector = 0.04150000*i_ + 0.00300000*j_ + 0*k_;
    const DQ x_effector = r_effector + E_*0.5*t_effector*r_effector;
    g1_left_arm.set_effector(x_effector);

    return g1_left_arm;
}

}
