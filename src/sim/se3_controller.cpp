#include "multi_quadcopter_control/sim/se3_controller.h"

#include <algorithm>
#include <cmath>

namespace mqc_sim {

namespace {

// d(v/|v|) = (dv |v| - v (v.dv)/|v|) / |v|^2
Eigen::Vector3d unitVectorDerivative(const Eigen::Vector3d& v, const Eigen::Vector3d& dv, double norm_v) {
    if (norm_v < 1e-6) return Eigen::Vector3d::Zero();
    return (dv * norm_v - v * (v.dot(dv) / norm_v)) / (norm_v * norm_v);
}

Eigen::Vector3d vee(const Eigen::Matrix3d& S) { return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0)); }

Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return S;
}

}  // namespace

Se3PositionController::Se3PositionController(const Se3PositionParams& p) : p_(p) {}

Se3PositionOutput Se3PositionController::compute(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                                                 const Eigen::Vector3d& ref_acc, const Eigen::Vector3d& ref_jerk,
                                                 double ref_psi, const Eigen::Vector3d& pos,
                                                 const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                                                 const Eigen::Matrix3d& R, const Eigen::Vector3d& f_dw) {
    const double m = p_.mass;
    const Eigen::Vector3d Kx(p_.kp_x, p_.kp_y, p_.kp_z);
    const Eigen::Vector3d Ki(p_.ki_x, p_.ki_y, p_.ki_z);
    const Eigen::Vector3d Kv(p_.kd_x, p_.kd_y, p_.kd_z);
    const Eigen::Vector3d e3(0.0, 0.0, 1.0);

    // Errors (actual - desired)
    const Eigen::Vector3d e_x = pos - ref_pos;
    const Eigen::Vector3d e_v = vel - ref_vel;
    integral_ += e_x * p_.dt;
    for (int i = 0; i < 3; ++i) integral_(i) = std::max(p_.integral_min, std::min(integral_(i), p_.integral_max));

    // Desired force
    Eigen::Vector3d F_des = -Kx.cwiseProduct(e_x) - Ki.cwiseProduct(integral_) - Kv.cwiseProduct(e_v)
                            + m * ref_acc + m * 9.81 * e3;
    if (p_.disturbance_feedforward) F_des -= f_dw;

    Se3PositionOutput out;
    // Thrust: projection of the desired force on the actual body z axis
    out.thrust = std::max(p_.min_thrust, std::min(F_des.dot(R.col(2)), p_.max_thrust));

    // Desired attitude
    const double norm_F = F_des.norm();
    const Eigen::Vector3d b3d = (norm_F < 1e-6) ? Eigen::Vector3d(prev_R_d_.col(2)) : Eigen::Vector3d(F_des / norm_F);
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

    // Desired angular velocity from the time derivative of F_des (d psi_d = 0)
    const Eigen::Vector3d dF_des = -Kx.cwiseProduct(e_v) - Ki.cwiseProduct(e_x)
                                   - Kv.cwiseProduct(acc - ref_acc) + m * ref_jerk;
    const Eigen::Vector3d db3d = unitVectorDerivative(F_des, dF_des, norm_F);
    const Eigen::Vector3d dC_vec = db3d.cross(b1c);
    const Eigen::Vector3d db2d = unitVectorDerivative(C_vec, dC_vec, norm_C);
    const Eigen::Vector3d db1d = db2d.cross(b3d) + b2d.cross(db3d);
    Eigen::Matrix3d dR_d;
    dR_d.col(0) = db1d;
    dR_d.col(1) = db2d;
    dR_d.col(2) = db3d;

    out.R_d = R_d;
    out.omega_d = vee(R_d.transpose() * dR_d);
    // Desired angular acceleration: finite difference at the controller rate
    out.omega_dot_d = first_ ? Eigen::Vector3d::Zero() : Eigen::Vector3d((out.omega_d - prev_omega_d_) / p_.dt);
    prev_omega_d_ = out.omega_d;
    first_ = false;
    return out;
}

Eigen::Vector3d Se3AttitudeController::compute(const Eigen::Matrix3d& R_d, const Eigen::Vector3d& omega_d,
                                               const Eigen::Vector3d& omega_dot_d, const Eigen::Matrix3d& R,
                                               const Eigen::Vector3d& omega) {
    const Eigen::Vector3d e_R = 0.5 * vee(R_d.transpose() * R - R.transpose() * R_d);
    const Eigen::Vector3d e_omega = omega - R.transpose() * R_d * omega_d;

    integral_ += e_R * p_.dt;
    for (int i = 0; i < 3; ++i) integral_(i) = std::max(p_.integral_min, std::min(integral_(i), p_.integral_max));

    const Eigen::Vector3d KR(p_.kp1, p_.kp2, p_.kp3);
    const Eigen::Vector3d KiR(p_.ki1, p_.ki2, p_.ki3);
    const Eigen::Vector3d KO(p_.kd1, p_.kd2, p_.kd3);

    const Eigen::Vector3d ff = p_.J * (hat(omega) * R.transpose() * R_d * omega_d - R.transpose() * R_d * omega_dot_d);
    return -KR.cwiseProduct(e_R) - KiR.cwiseProduct(integral_) - KO.cwiseProduct(e_omega)
           + omega.cross(p_.J * omega) - ff;
}

Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& R) {
    return Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)), -std::asin(R(2, 0)), std::atan2(R(1, 0), R(0, 0)));
}

}  // namespace mqc_sim
