#include "multi_quadcopter_control/sim/se3_controller.h"

#include <algorithm>
#include <cmath>

namespace mqc_sim {

namespace {

// MidLevelNS::computePID of the SE(3) package (error = actual - desired)
double se3PID(double des_pos, double a_pos, double des_vel, double a_vel, double& integral,
              double kp, double ki, double kd, double dt, double integral_min, double integral_max) {
    double pos_error = a_pos - des_pos;
    double vel_error = a_vel - des_vel;
    integral += pos_error * dt;
    if (integral > integral_max) integral = integral_max;
    else if (integral < integral_min) integral = integral_min;
    return kp * pos_error + ki * integral + kd * vel_error;
}

// d(v/|v|) = (dv |v| - v (v.dv)/|v|) / |v|^2
Eigen::Vector3d unitVectorDerivative(const Eigen::Vector3d& v, const Eigen::Vector3d& dv, double norm_v) {
    if (norm_v < 1e-6) return Eigen::Vector3d::Zero();
    const double v_dot_dv = v.dot(dv);
    return (dv * norm_v - v * (v_dot_dv / norm_v)) / (norm_v * norm_v);
}

Eigen::Vector3d vee(const Eigen::Matrix3d& S) { return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0)); }

}  // namespace

Se3PositionController::Se3PositionController(const Se3PositionParams& p) : p_(p) {}

Se3PositionOutput Se3PositionController::compute(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                                                 const Eigen::Vector3d& ref_acc, const Eigen::Vector3d& ref_jerk,
                                                 double ref_psi, const Eigen::Vector3d& pos,
                                                 const Eigen::Vector3d& vel, const Eigen::Vector3d& f_dw) {
    const double m = p_.mass;

    // Step 1: desired force (computeThrust)
    const double pid_x = se3PID(ref_pos.x(), pos.x(), ref_vel.x(), vel.x(), integral_.x(),
                                p_.kp_x, p_.ki_x, p_.kd_x, p_.dt, p_.integral_min, p_.integral_max);
    const double pid_y = se3PID(ref_pos.y(), pos.y(), ref_vel.y(), vel.y(), integral_.y(),
                                p_.kp_y, p_.ki_y, p_.kd_y, p_.dt, p_.integral_min, p_.integral_max);
    const double pid_z = se3PID(ref_pos.z(), pos.z(), ref_vel.z(), vel.z(), integral_.z(),
                                p_.kp_z, p_.ki_z, p_.kd_z, p_.dt, p_.integral_min, p_.integral_max);

    double f_x = -pid_x + m * ref_acc.x();
    double f_y = -pid_y + m * ref_acc.y();
    double f_z = -pid_z + m * ref_acc.z() + m * 9.81;
    if (p_.disturbance_feedforward) {
        f_x -= f_dw.x();
        f_y -= f_dw.y();
        f_z -= f_dw.z();
    }
    const Eigen::Vector3d F_des(f_x, f_y, f_z);

    Se3PositionOutput out;
    out.thrust = std::max(p_.min_thrust, std::min(F_des.norm(), p_.max_thrust));

    // Step 2: desired attitude (computeAndPublishRotation)
    const double norm_thrust = F_des.norm();
    const Eigen::Vector3d b3d = (norm_thrust < 1e-6) ? Eigen::Vector3d(prev_R_d_.col(2)) : F_des.normalized();
    const Eigen::Vector3d b1c(std::cos(ref_psi), std::sin(ref_psi), 0.0);
    const Eigen::Vector3d C_vec = b3d.cross(b1c);
    const double norm_C = C_vec.norm();
    const Eigen::Vector3d b2d = (norm_C < 1e-6) ? Eigen::Vector3d(prev_R_d_.col(1)) : Eigen::Vector3d(C_vec / norm_C);
    const Eigen::Vector3d b1d = b2d.cross(b3d);

    Eigen::Matrix3d R_d;
    R_d.col(0) = b1d;
    R_d.col(1) = b2d;
    R_d.col(2) = b3d;
    prev_R_d_ = R_d;

    // Desired angular velocity: dF_des = -Kp e_v + m j_des (Kd term neglected, d psi_des = 0)
    const Eigen::Vector3d err_v = vel - ref_vel;
    const Eigen::Vector3d dF_des(-p_.kp_x * err_v.x() + m * ref_jerk.x(),
                                 -p_.kp_y * err_v.y() + m * ref_jerk.y(),
                                 -p_.kp_z * err_v.z() + m * ref_jerk.z());
    const double ref_d_psi = 0.0;  // reference_d_psi is never published in the original package

    const Eigen::Vector3d db3d = unitVectorDerivative(F_des, dF_des, norm_thrust);
    const Eigen::Vector3d db1c(-std::sin(ref_psi) * ref_d_psi, std::cos(ref_psi) * ref_d_psi, 0.0);
    const Eigen::Vector3d dC_vec = db3d.cross(b1c) + b3d.cross(db1c);
    const Eigen::Vector3d db2d = unitVectorDerivative(C_vec, dC_vec, norm_C);
    const Eigen::Vector3d db1d = db2d.cross(b3d) + b2d.cross(db3d);

    Eigen::Matrix3d dR_d;
    dR_d.col(0) = db1d;
    dR_d.col(1) = db2d;
    dR_d.col(2) = db3d;
    const Eigen::Matrix3d S_omega = R_d.transpose() * dR_d;

    out.R_d = R_d;
    out.omega_d = Eigen::Vector3d(S_omega(2, 1), S_omega(0, 2), S_omega(1, 0));
    return out;
}

Eigen::Vector3d Se3AttitudeController::compute(const Eigen::Matrix3d& R_d, const Eigen::Vector3d& omega_d,
                                               const Eigen::Matrix3d& R, const Eigen::Vector3d& omega) {
    const Eigen::Vector3d e_R = 0.5 * vee(R_d.transpose() * R - R.transpose() * R_d);
    const Eigen::Vector3d e_omega = omega - (R.transpose() * R_d * omega_d);

    integral_ += e_R * p_.dt;
    for (int i = 0; i < 3; ++i) {
        if (integral_(i) > p_.integral_max) integral_(i) = p_.integral_max;
        else if (integral_(i) < p_.integral_min) integral_(i) = p_.integral_min;
    }

    const Eigen::Vector3d Kp(p_.kp1, p_.kp2, p_.kp3);
    const Eigen::Vector3d Ki(p_.ki1, p_.ki2, p_.ki3);
    const Eigen::Vector3d Kd(p_.kd1, p_.kd2, p_.kd3);
    const Eigen::Vector3d gyro = omega.cross(p_.J * omega);

    return -Kp.cwiseProduct(e_R) - Ki.cwiseProduct(integral_) - Kd.cwiseProduct(e_omega) + gyro;
}

Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& R) {
    return Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)), -std::asin(R(2, 0)), std::atan2(R(1, 0), R(0, 0)));
}

}  // namespace mqc_sim
