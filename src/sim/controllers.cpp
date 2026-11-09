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

// Same as LowLevelNS::computePID. wrap_angle: wrap the error to [-pi, pi] (yaw), otherwise a reference
// near -pi and a measurement near +pi give a ~2 pi error and the controller turns the wrong way.
double lowPID(double setpoint, double measured_value, double& prev_error, double& integral,
              double kp, double ki, double kd, double dt, double integral_min, double integral_max,
              bool wrap_angle = false) {
    double error = setpoint - measured_value;
    if (wrap_angle) error = std::remainder(error, 2.0 * M_PI);  // unchanged when |error| < pi

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
    // All modes share the same hierarchical PID law; they differ only in the disturbance
    // compensation term f_comp subtracted from the desired force:
    //   PID (no feedforward): f_comp = 0
    //   PID + feedforward   : f_comp = f_dw      (modelled downwash, Eq. 21)
    //   NDOB                : f_comp = delta_f   (disturbance estimated by the observer)
    Eigen::Vector3d f_comp = Eigen::Vector3d::Zero();
    bool use_comp = false;
    if (p_.disturbance_feedforward) {
        f_comp += f_dw;
        use_comp = true;
    }
    if (p_.mode == MidLevelMode::NDOB) {
        updateObserver(vel, euler);
        f_comp += delta_f_;
        use_comp = true;
    }

    MidLevelOutput out = computeLaw(ref_pos, ref_vel, ref_acc, ref_psi, pos, vel, euler, f_comp, use_comp);

    if (p_.mode == MidLevelMode::NDOB) {
        last_thrust_ = out.thrust;  // observer input at the next step
        out.ndob_estimate = delta_f_;
    }
    return out;
}

// Hierarchical PID (src/mid_level_cont.cpp) with an optional compensation term
MidLevelOutput MidLevelController::computeLaw(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                                              const Eigen::Vector3d& ref_acc, double ref_psi,
                                              const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                                              const Eigen::Vector3d& euler, const Eigen::Vector3d& f_comp,
                                              bool use_comp) {
    const double m = p_.mass;

    // World-frame PID forces (MidLevelController::computeThrust)
    double thrust_x = midPID(ref_pos.x(), pos.x(), ref_vel.x(), vel.x(), integral_x_,
                             p_.kp_x, p_.ki_x, p_.kd_x, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.x();
    double thrust_y = midPID(ref_pos.y(), pos.y(), ref_vel.y(), vel.y(), integral_y_,
                             p_.kp_y, p_.ki_y, p_.kd_y, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.y();
    double thrust_z = midPID(ref_pos.z(), pos.z(), ref_vel.z(), vel.z(), integral_z_,
                             p_.kp_z, p_.ki_z, p_.kd_z, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.z() + m * 9.81;

    // Disturbance compensation: f_des = Kp e_p + Kd e_v + m a_des + m g - f_comp
    if (use_comp) {
        thrust_x -= f_comp.x();
        thrust_y -= f_comp.y();
        thrust_z -= f_comp.z();
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

// Model-based nonlinear disturbance observer (translational dynamics, world frame):
//   delta_f = z + L m v,   z_dot = -L delta_f - L (f_thrust_world - m g e3)
// f_thrust_world is the previous thrust command passed through a first-order filter with the
// actuator time constant (ndob_tau_motor = tau_thrust), projected with the measured attitude.
void MidLevelController::updateObserver(const Eigen::Vector3d& vel, const Eigen::Vector3d& euler) {
    const double m = p_.mass;
    const double dt = p_.dt;
    const double phi = euler.x(), theta = euler.y(), psi = euler.z();

    const double alpha = dt / (p_.ndob_tau_motor + dt);
    filtered_thrust_ = (1.0 - alpha) * filtered_thrust_ + alpha * last_thrust_;

    const double c_phi = std::cos(phi), s_phi = std::sin(phi);
    const double c_theta = std::cos(theta), s_theta = std::sin(theta);
    const double c_psi = std::cos(psi), s_psi = std::sin(psi);

    const double f_thrust_world_x = filtered_thrust_ * (c_psi * s_theta * c_phi + s_psi * s_phi);
    const double f_thrust_world_y = filtered_thrust_ * (s_psi * s_theta * c_phi - c_psi * s_phi);
    const double f_thrust_world_z = filtered_thrust_ * (c_theta * c_phi);

    // Estimate (before integration)
    delta_f_.x() = zv_.x() + p_.lv_x * m * vel.x();
    delta_f_.y() = zv_.y() + p_.lv_y * m * vel.y();
    delta_f_.z() = zv_.z() + p_.lv_z * m * vel.z();

    // Observer dynamics, explicit Euler
    zv_.x() += (-p_.lv_x * delta_f_.x() - p_.lv_x * f_thrust_world_x) * dt;
    zv_.y() += (-p_.lv_y * delta_f_.y() - p_.lv_y * f_thrust_world_y) * dt;
    zv_.z() += (-p_.lv_z * delta_f_.z() - p_.lv_z * (f_thrust_world_z - m * 9.81)) * dt;
}

Eigen::Vector3d LowLevelController::compute(const Eigen::Vector3d& ref_angles, const Eigen::Vector3d& euler) {
    double t1 = lowPID(ref_angles.x(), euler.x(), prev_error_.x(), integral_.x(),
                       p_.kp1, p_.ki1, p_.kd1, p_.dt, p_.integral_min, p_.integral_max);
    double t2 = lowPID(ref_angles.y(), euler.y(), prev_error_.y(), integral_.y(),
                       p_.kp2, p_.ki2, p_.kd2, p_.dt, p_.integral_min, p_.integral_max);
    double t3 = lowPID(ref_angles.z(), euler.z(), prev_error_.z(), integral_.z(),
                       p_.kp3, p_.ki3, p_.kd3, p_.dt, p_.integral_min, p_.integral_max, true);
    return Eigen::Vector3d(t1, t2, t3);
}

}  // namespace mqc_sim
