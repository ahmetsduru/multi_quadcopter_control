// Geometric (SE(3)) controller (ROS-free), ported from
// github.com/ahmetsduru/multi_quadcopter_geometric_control (SE3_mid_level_cont.cpp, SE3_low_level_cont.cpp):
//   position : F_des = -(Kp e_p + Ki int(e_p) + Kd e_v) + m a_des + m g - f_dw,  e = actual - desired
//              thrust = sat(||F_des||), R_d from b3d = F_des/||F_des|| and b1c(psi_des),
//              Omega_d from dF_des = -Kp e_v + m j_des (d psi_des = 0, as in the original)
//   attitude : tau = -Kp e_R - Ki int(e_R) - Kd e_Omega + Omega x J Omega
//              e_R = 1/2 vee(R_d^T R - R^T R_d), e_Omega = Omega - R^T R_d Omega_d
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
};

class Se3PositionController {
public:
    explicit Se3PositionController(const Se3PositionParams& p);
    Se3PositionOutput compute(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                              const Eigen::Vector3d& ref_acc, const Eigen::Vector3d& ref_jerk, double ref_psi,
                              const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                              const Eigen::Vector3d& f_dw);

private:
    Se3PositionParams p_;
    Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d prev_R_d_ = Eigen::Matrix3d::Identity();
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
                            const Eigen::Matrix3d& R, const Eigen::Vector3d& omega);

private:
    Se3AttitudeParams p_;
    Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
};

// ZYX Euler angles (phi, theta, psi) of a rotation matrix (same convention as the dynamics)
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& R);

}  // namespace mqc_sim

#endif  // MQC_SIM_SE3_CONTROLLER_H
