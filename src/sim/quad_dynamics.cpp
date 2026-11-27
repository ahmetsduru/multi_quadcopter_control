#include "multi_quadcopter_control/sim/quad_dynamics.h"

#include <boost/numeric/odeint.hpp>
#include <cmath>

namespace mqc_sim {

QuadDynamics::QuadDynamics(const QuadParams& params) : p_(params) {
    I_inv_ = p_.I.inverse();
    x_ = std::vector<double>(18, 0.0);
    x_[3] = x_[7] = x_[11] = 1.0;           // R = I
    thrust_actual_ = p_.mass * 9.81;         // Hover thrust
    torques_actual_.setZero();
    t_ = 0.0;
}

void QuadDynamics::setThrustCommand(double thrust) {
    thrust_cmd_ = thrust;
    thrust_received_ = true;
}

void QuadDynamics::setTorqueCommand(const Eigen::Vector3d& torques) {
    torques_cmd_ = torques;
}

void QuadDynamics::step(const Eigen::Vector3d& disturbance_force) {
    const double dt = p_.dt;

    // First-order actuator lags (same explicit update as eom.cpp)
    if (thrust_received_) {
        thrust_actual_ += (1.0 / p_.tau_thrust) * (thrust_cmd_ - thrust_actual_) * dt;
    } else {
        thrust_actual_ = p_.mass * 9.81;  // Hover thrust until the first command arrives
    }
    torques_actual_ += (1.0 / p_.tau_torque) * (torques_cmd_ - torques_actual_) * dt;

    const Eigen::Vector3d thrust_vec(0, 0, thrust_actual_);
    const Eigen::Vector3d tau = torques_actual_;

    using namespace boost::numeric::odeint;
    typedef runge_kutta_cash_karp54<std::vector<double>> error_stepper_type;
    typedef controlled_runge_kutta<error_stepper_type> controlled_stepper_type;

    integrate_adaptive(
        controlled_stepper_type(),
        [&](const std::vector<double>& x, std::vector<double>& dxdt, double /*t*/) {
            computeDerivatives(x, dxdt, thrust_vec, tau, disturbance_force);
        },
        x_, t_, t_ + dt, dt);

    orthonormalizeRotation();
    t_ += dt;
}

Eigen::Vector3d QuadDynamics::position() const { return Eigen::Vector3d(x_[0], x_[1], x_[2]); }
Eigen::Vector3d QuadDynamics::velocity() const { return Eigen::Vector3d(x_[12], x_[13], x_[14]); }
Eigen::Vector3d QuadDynamics::angularVelocity() const { return Eigen::Vector3d(x_[15], x_[16], x_[17]); }

Eigen::Matrix3d QuadDynamics::rotation() const {
    Eigen::Matrix3d R;
    R << x_[3], x_[4], x_[5],
         x_[6], x_[7], x_[8],
         x_[9], x_[10], x_[11];
    return R;
}

Eigen::Vector3d QuadDynamics::eulerAngles() const {
    const Eigen::Matrix3d R = rotation();
    return Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)),
                           -std::asin(R(2, 0)),
                           std::atan2(R(1, 0), R(0, 0)));
}

// f_drag = -1/2 rho C_d A ||v|| v (Eq. 14). The previous form -1/2 rho C_d A (v.*v) always pointed in
// the negative axis directions, regardless of the direction of motion.
Eigen::Vector3d QuadDynamics::drag(const Eigen::Vector3d& V) const {
    return -0.5 * p_.rho * (p_.Cd_trans * p_.A_trans * V) * V.norm();
}

Eigen::Vector3d QuadDynamics::acceleration(const Eigen::Vector3d& disturbance_force) const {
    const Eigen::Vector3d thrust_vec(0, 0, thrust_actual_);
    return (rotation() * thrust_vec + drag(velocity()) + p_.mass * p_.g + disturbance_force) / p_.mass;
}

void QuadDynamics::computeDerivatives(const std::vector<double>& x, std::vector<double>& dxdt,
                                      const Eigen::Vector3d& thrust_vec, const Eigen::Vector3d& tau,
                                      const Eigen::Vector3d& disturbance) const {
    dxdt.resize(18);
    Eigen::Matrix3d R;
    R << x[3], x[4], x[5],
         x[6], x[7], x[8],
         x[9], x[10], x[11];
    Eigen::Vector3d V(x[12], x[13], x[14]), w(x[15], x[16], x[17]);

    for (int i = 0; i < 3; ++i) dxdt[i] = V[i];
    Eigen::Matrix3d dR = R * skew(w);
    for (int i = 0; i < 9; ++i) dxdt[3 + i] = dR(i / 3, i % 3);

    Eigen::Vector3d acc = (R * thrust_vec + drag(V) + p_.mass * p_.g + disturbance) / p_.mass;
    for (int i = 0; i < 3; ++i) dxdt[12 + i] = acc[i];

    Eigen::Vector3d dw = I_inv_ * (tau - skew(w) * p_.I * w);
    for (int i = 0; i < 3; ++i) dxdt[15 + i] = dw[i];
}

void QuadDynamics::orthonormalizeRotation() {
    Eigen::Matrix3d R = rotation();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    for (int i = 0; i < 9; ++i) x_[3 + i] = R(i / 3, i % 3);
}

Eigen::Matrix3d QuadDynamics::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d s;
    s << 0, -v[2], v[1],
         v[2], 0, -v[0],
        -v[1], v[0], 0;
    return s;
}

}  // namespace mqc_sim
