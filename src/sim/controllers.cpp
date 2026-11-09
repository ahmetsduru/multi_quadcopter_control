#include "multi_quadcopter_control/sim/controllers.h"

#include <algorithm>
#include <cmath>

namespace mqc_sim {

namespace {

// Same as MidLevelNS::computePID (PD on position/velocity error + clamped integral)
double midPID(double des_pos, double a_pos, double des_vel, double a_vel, double& integral,
              double kp, double ki, double kd, double dt, double integral_min, double integral_max) {
    double pos_error = des_pos - a_pos;
    double vel_error = des_vel - a_vel;

    integral += pos_error * dt;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }
    return kp * pos_error + ki * integral + kd * vel_error;
}

// Same as LowLevelNS::computePID
double lowPID(double setpoint, double measured_value, double& prev_error, double& integral,
              double kp, double ki, double kd, double dt, double integral_min, double integral_max) {
    double error = setpoint - measured_value;

    integral += error * dt;
    if (integral > integral_max) {
        integral = integral_max;
    } else if (integral < integral_min) {
        integral = integral_min;
    }

    double derivative = (error - prev_error) / dt;
    prev_error = error;

    return kp * error + ki * integral + kd * derivative;
}

}  // namespace

MidLevelOutput MidLevelController::compute(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                                           const Eigen::Vector3d& ref_acc, double ref_psi,
                                           const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                                           const Eigen::Vector3d& euler, const Eigen::Vector3d& f_dw) {
    if (p_.mode == MidLevelMode::NDOB) {
        return computeNDOB(ref_pos, ref_vel, ref_acc, ref_psi, pos, vel, euler, f_dw);
    }
    return computePID(ref_pos, ref_vel, ref_acc, ref_psi, pos, vel, euler, f_dw);
}

// Hierarchical PID (current src/mid_level_cont.cpp)
MidLevelOutput MidLevelController::computePID(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                                              const Eigen::Vector3d& ref_acc, double ref_psi,
                                              const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                                              const Eigen::Vector3d& euler, const Eigen::Vector3d& f_dw) {
    const double m = p_.mass;

    // World-frame PID forces (MidLevelController::computeThrust)
    double thrust_x = midPID(ref_pos.x(), pos.x(), ref_vel.x(), vel.x(), integral_x_,
                             p_.kp_x, p_.ki_x, p_.kd_x, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.x();
    double thrust_y = midPID(ref_pos.y(), pos.y(), ref_vel.y(), vel.y(), integral_y_,
                             p_.kp_y, p_.ki_y, p_.kd_y, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.y();
    double thrust_z = midPID(ref_pos.z(), pos.z(), ref_vel.z(), vel.z(), integral_z_,
                             p_.kp_z, p_.ki_z, p_.kd_z, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.z() + m * 9.81;

    // Disturbance feedforward: f_des = Kp e_p + Kd e_v + m a_des + m g - f_dw
    if (p_.disturbance_feedforward) {
        thrust_x -= f_dw.x();
        thrust_y -= f_dw.y();
        thrust_z -= f_dw.z();
    }

    const double phi = euler.x(), theta = euler.y(), psi = euler.z();

    double projection_denominator = std::cos(phi) * std::cos(theta);
    if (projection_denominator < 0.1) {
        projection_denominator = 0.1;
    }
    double total_thrust = thrust_z / projection_denominator;

    MidLevelOutput out;
    out.thrust = std::max(p_.min_thrust, std::min(total_thrust, p_.max_thrust));

    // Reference angles (MidLevelController::computeReferenceAngles)
    double safe_thrust_z = thrust_z;
    if (safe_thrust_z < 1.0) {
        safe_thrust_z = 1.0;
    }
    double ref_phi = (thrust_x * std::sin(psi) - thrust_y * std::cos(psi)) / safe_thrust_z;
    double ref_theta = (thrust_x * std::cos(psi) + thrust_y * std::sin(psi)) / safe_thrust_z;

    const double angle_limit = 0.6;
    ref_phi = std::max(-angle_limit, std::min(ref_phi, angle_limit));
    ref_theta = std::max(-angle_limit, std::min(ref_theta, angle_limit));

    out.ref_angles = Eigen::Vector3d(ref_phi, ref_theta, ref_psi);
    return out;
}

// PID + model-based NDOB (NDOB version of src/mid_level_cont.cpp, same equations and order)
MidLevelOutput MidLevelController::computeNDOB(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                                               const Eigen::Vector3d& ref_acc, double ref_psi,
                                               const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                                               const Eigen::Vector3d& euler, const Eigen::Vector3d& f_dw) {
    const double m = p_.mass;
    const double dt = p_.dt;
    const double phi = euler.x(), theta = euler.y(), psi = euler.z();

    // Thrust filter (actuator lag seen by the observer)
    const double alpha = dt / (p_.ndob_tau_motor + dt);
    filtered_thrust_ = (1.0 - alpha) * filtered_thrust_ + alpha * last_thrust_;

    const double c_phi = std::cos(phi), s_phi = std::sin(phi);
    const double c_theta = std::cos(theta), s_theta = std::sin(theta);
    const double c_psi = std::cos(psi), s_psi = std::sin(psi);

    // Body-z thrust projected to the world frame
    const double f_thrust_world_x = filtered_thrust_ * (c_psi * s_theta * c_phi + s_psi * s_phi);
    const double f_thrust_world_y = filtered_thrust_ * (s_psi * s_theta * c_phi - c_psi * s_phi);
    const double f_thrust_world_z = filtered_thrust_ * (c_theta * c_phi);

    // Disturbance estimate (before integration)
    delta_f_.x() = zv_.x() + p_.lv_x * m * vel.x();
    delta_f_.y() = zv_.y() + p_.lv_y * m * vel.y();
    delta_f_.z() = zv_.z() + p_.lv_z * m * vel.z();

    // Observer dynamics + Euler integration
    const double dz_x = -p_.lv_x * delta_f_.x() - p_.lv_x * f_thrust_world_x;
    const double dz_y = -p_.lv_y * delta_f_.y() - p_.lv_y * f_thrust_world_y;
    const double dz_z = -p_.lv_z * delta_f_.z() - p_.lv_z * (f_thrust_world_z - (m * 9.81));
    zv_.x() += dz_x * dt;
    zv_.y() += dz_y * dt;
    zv_.z() += dz_z * dt;

    // Control law: PID + feedforward - estimated disturbance
    double thrust_x = midPID(ref_pos.x(), pos.x(), ref_vel.x(), vel.x(), integral_x_,
                             p_.kp_x, p_.ki_x, p_.kd_x, dt, p_.integral_min, p_.integral_max)
                      + (m * ref_acc.x()) - delta_f_.x();
    double thrust_y = midPID(ref_pos.y(), pos.y(), ref_vel.y(), vel.y(), integral_y_,
                             p_.kp_y, p_.ki_y, p_.kd_y, dt, p_.integral_min, p_.integral_max)
                      + (m * ref_acc.y()) - delta_f_.y();
    double thrust_z = midPID(ref_pos.z(), pos.z(), ref_vel.z(), vel.z(), integral_z_,
                             p_.kp_z, p_.ki_z, p_.kd_z, dt, p_.integral_min, p_.integral_max)
                      + (m * ref_acc.z()) + (m * 9.81) - delta_f_.z();

    // Optional modelled-downwash feedforward (off by default)
    if (p_.disturbance_feedforward) {
        thrust_x -= f_dw.x();
        thrust_y -= f_dw.y();
        thrust_z -= f_dw.z();
    }

    // Reference angles (exact, nonlinear transformation)
    Eigen::Vector3d ref_angles;
    if (std::abs(thrust_z) < 1e-5) {
        ref_angles = Eigen::Vector3d(0.0, 0.0, ref_psi);
    } else {
        const double total = std::sqrt(thrust_x * thrust_x + thrust_y * thrust_y + thrust_z * thrust_z);
        double sin_phi_val = (thrust_x * std::sin(psi) - thrust_y * std::cos(psi)) / total;
        sin_phi_val = std::max(-1.0, std::min(1.0, sin_phi_val));
        ref_angles = Eigen::Vector3d(std::asin(sin_phi_val),
                                     std::atan2(thrust_x * std::cos(psi) + thrust_y * std::sin(psi), thrust_z),
                                     ref_psi);
    }

    // World forces -> body frame with the reference attitude; only body-z thrust can be applied
    const double phi_r = ref_angles.x(), theta_r = ref_angles.y(), psi_r = ref_angles.z();
    Eigen::Matrix3d R;
    R << std::cos(psi_r) * std::cos(theta_r), std::sin(psi_r) * std::cos(theta_r), -std::sin(theta_r),
         std::cos(psi_r) * std::sin(theta_r) * std::sin(phi_r) - std::sin(psi_r) * std::cos(phi_r),
         std::sin(psi_r) * std::sin(theta_r) * std::sin(phi_r) + std::cos(psi_r) * std::cos(phi_r),
         std::cos(theta_r) * std::sin(phi_r),
         std::cos(psi_r) * std::sin(theta_r) * std::cos(phi_r) + std::sin(psi_r) * std::sin(phi_r),
         std::sin(psi_r) * std::sin(theta_r) * std::cos(phi_r) - std::cos(psi_r) * std::sin(phi_r),
         std::cos(theta_r) * std::cos(phi_r);
    const Eigen::Vector3d body_forces = R * Eigen::Vector3d(thrust_x, thrust_y, thrust_z);
    last_thrust_ = std::max(p_.min_thrust, std::min(body_forces.z(), p_.max_thrust));

    MidLevelOutput out;
    out.thrust = last_thrust_;
    out.ref_angles = ref_angles;
    out.ndob_estimate = delta_f_;
    return out;
}

Eigen::Vector3d LowLevelController::compute(const Eigen::Vector3d& ref_angles, const Eigen::Vector3d& euler) {
    double t1 = lowPID(ref_angles.x(), euler.x(), prev_error_.x(), integral_.x(),
                       p_.kp1, p_.ki1, p_.kd1, p_.dt, p_.integral_min, p_.integral_max);
    double t2 = lowPID(ref_angles.y(), euler.y(), prev_error_.y(), integral_.y(),
                       p_.kp2, p_.ki2, p_.kd2, p_.dt, p_.integral_min, p_.integral_max);
    double t3 = lowPID(ref_angles.z(), euler.z(), prev_error_.z(), integral_.z(),
                       p_.kp3, p_.ki3, p_.kd3, p_.dt, p_.integral_min, p_.integral_max);
    return Eigen::Vector3d(t1, t2, t3);
}

}  // namespace mqc_sim
