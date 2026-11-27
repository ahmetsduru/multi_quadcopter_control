// Quadcopter rigid-body dynamics (ROS-free).
// Same equations of motion, actuator lags and integrator as src/eom.cpp.
#ifndef MQC_SIM_QUAD_DYNAMICS_H
#define MQC_SIM_QUAD_DYNAMICS_H

#include <Eigen/Dense>
#include <vector>

namespace mqc_sim {

struct QuadParams {
    double mass = 0.382;
    double rho = 1.225;
    double tau_thrust = 0.1;
    double tau_torque = 0.2;
    double dt = 0.002;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Cd_trans = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d A_trans = Eigen::Matrix3d::Zero();
    Eigen::Vector3d g = Eigen::Vector3d(0.0, 0.0, -9.81);
};

class QuadDynamics {
public:
    explicit QuadDynamics(const QuadParams& params);

    // Latest commands from the controllers (held until the next call).
    void setThrustCommand(double thrust);
    void setTorqueCommand(const Eigen::Vector3d& torques);

    // Advance the state by one time step (params.dt) with the given external force (world frame).
    void step(const Eigen::Vector3d& disturbance_force);

    // State accessors (body position is relative to the drone's initial position).
    Eigen::Vector3d position() const;
    Eigen::Vector3d velocity() const;
    Eigen::Vector3d angularVelocity() const;
    Eigen::Matrix3d rotation() const;
    Eigen::Vector3d eulerAngles() const;  // (phi, theta, psi), same convention as eom.cpp
    // Linear acceleration for the current state, actual thrust and the given disturbance.
    Eigen::Vector3d acceleration(const Eigen::Vector3d& disturbance_force) const;

    double thrustActual() const { return thrust_actual_; }
    double time() const { return t_; }
    const QuadParams& params() const { return p_; }

private:
    QuadParams p_;
    Eigen::Matrix3d I_inv_;
    std::vector<double> x_;  // [pos(3), R row-major(9), vel(3), omega(3)]
    double t_ = 0.0;

    double thrust_cmd_ = 0.0;
    double thrust_actual_ = 0.0;
    bool thrust_received_ = false;
    Eigen::Vector3d torques_cmd_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d torques_actual_ = Eigen::Vector3d::Zero();

    void computeDerivatives(const std::vector<double>& x, std::vector<double>& dxdt,
                            const Eigen::Vector3d& thrust_vec, const Eigen::Vector3d& tau,
                            const Eigen::Vector3d& disturbance) const;
    void orthonormalizeRotation();
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    Eigen::Vector3d drag(const Eigen::Vector3d& V) const;
};

}  // namespace mqc_sim

#endif  // MQC_SIM_QUAD_DYNAMICS_H
