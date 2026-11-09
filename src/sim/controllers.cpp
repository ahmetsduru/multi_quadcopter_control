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
                                           const Eigen::Vector3d& euler) {
    const double m = p_.mass;

    // World-frame PID forces (MidLevelController::computeThrust)
    double thrust_x = midPID(ref_pos.x(), pos.x(), ref_vel.x(), vel.x(), integral_x_,
                             p_.kp_x, p_.ki_x, p_.kd_x, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.x();
    double thrust_y = midPID(ref_pos.y(), pos.y(), ref_vel.y(), vel.y(), integral_y_,
                             p_.kp_y, p_.ki_y, p_.kd_y, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.y();
    double thrust_z = midPID(ref_pos.z(), pos.z(), ref_vel.z(), vel.z(), integral_z_,
                             p_.kp_z, p_.ki_z, p_.kd_z, p_.dt, p_.integral_min, p_.integral_max) + m * ref_acc.z() + m * 9.81;

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
