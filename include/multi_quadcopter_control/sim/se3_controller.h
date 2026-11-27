// Geometric tracking controller on SE(3), Lee, Leok and McClamroch (CDC 2010), ROS-free.
// World frame with z up (g = -9.81 e3), errors e = actual - desired:
//   F_des = -K_x e_x - K_i int(e_x) - K_v e_v + m a_d + m g e3 - f_dw     (f_dw: optional downwash feedforward)
//   f     = F_des . R e3                                                  (thrust along the actual body z axis)
//   b3d = F_des/|F_des|, b1c = (cos psi_d, sin psi_d, 0), b2d = b3d x b1c / |.|, b1d = b2d x b3d
//   Omega_d = vee(R_d^T dR_d), dR_d from dF_des = -K_x e_v - K_i e_x - K_v (a - a_d) + m j_d
//   M = -K_R e_R - K_iR int(e_R) - K_Omega e_Omega + Omega x J Omega
//       - J (hat(Omega) R^T R_d Omega_d - R^T R_d dOmega_d)
//   e_R = 1/2 vee(R_d^T R - R^T R_d), e_Omega = Omega - R^T R_d Omega_d
// Practical choices: dOmega_d is the finite difference of Omega_d at the position-controller rate, and
// d psi_d = 0 (the yaw reference is a filtered discrete-time signal). The integral gains are 0 in the
// parameter files (as in Lee et al.).
#ifndef MQC_SIM_SE3_CONTROLLER_H
#define MQC_SIM_SE3_CONTROLLER_H

#include <Eigen/Dense>

namespace mqc_sim {

struct Se3PositionParams {
    double kp_x = 0, ki_x = 0, kd_x = 0;
    double kp_y = 0, ki_y = 0, kd_y = 0;
    double kp_z = 0, ki_z = 0, kd_z = 0;
    double integral_min = 0, integral_max = 0;
    double min_thrust = 0, max_thrust = 0;
    double dt = 0.01;
    double mass = 0.382;
    bool disturbance_feedforward = true;  // f_dw (modelled downwash at the actual positions)
};

struct Se3PositionOutput {
    double thrust = 0.0;
    Eigen::Matrix3d R_d = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega_d = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega_dot_d = Eigen::Vector3d::Zero();
};

class Se3PositionController {
public:
    explicit Se3PositionController(const Se3PositionParams& p);
    // acc: measured linear acceleration (world frame), R: measured attitude
    Se3PositionOutput compute(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                              const Eigen::Vector3d& ref_acc, const Eigen::Vector3d& ref_jerk, double ref_psi,
                              const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                              const Eigen::Vector3d& acc, const Eigen::Matrix3d& R,
                              const Eigen::Vector3d& f_dw);

private:
    Se3PositionParams p_;
    Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d prev_R_d_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d prev_omega_d_ = Eigen::Vector3d::Zero();
    bool first_ = true;
};

struct Se3AttitudeParams {
    double kp1 = 0, ki1 = 0, kd1 = 0;
    double kp2 = 0, ki2 = 0, kd2 = 0;
    double kp3 = 0, ki3 = 0, kd3 = 0;
    double integral_min = 0, integral_max = 0;
    double dt = 0.004;
    Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
};

class Se3AttitudeController {
public:
    explicit Se3AttitudeController(const Se3AttitudeParams& p) : p_(p) {}
    Eigen::Vector3d compute(const Eigen::Matrix3d& R_d, const Eigen::Vector3d& omega_d,
                            const Eigen::Vector3d& omega_dot_d, const Eigen::Matrix3d& R,
                            const Eigen::Vector3d& omega);

private:
    Se3AttitudeParams p_;
    Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
};

// ZYX Euler angles (phi, theta, psi) of a rotation matrix (same convention as the dynamics)
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& R);

}  // namespace mqc_sim

#endif  // MQC_SIM_SE3_CONTROLLER_H
