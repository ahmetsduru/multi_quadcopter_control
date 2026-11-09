// Mid-level (position) and low-level (attitude) controllers (ROS-free).
// Same control laws as src/mid_level_cont.cpp and src/low_level_cont.cpp.
#ifndef MQC_SIM_CONTROLLERS_H
#define MQC_SIM_CONTROLLERS_H

#include <Eigen/Dense>

namespace mqc_sim {

// ---------------------------------------------------------------- Mid level
struct MidLevelParams {
    double kp_x = 0, ki_x = 0, kd_x = 0;
    double kp_y = 0, ki_y = 0, kd_y = 0;
    double kp_z = 0, ki_z = 0, kd_z = 0;
    double integral_min = 0, integral_max = 0;
    double min_thrust = 0, max_thrust = 0;
    double dt = 0.05;
    double mass = 0.382;
};

struct MidLevelOutput {
    double thrust = 0.0;                                     // reference_thrust
    Eigen::Vector3d ref_angles = Eigen::Vector3d::Zero();    // reference_euler_angles
};

class MidLevelController {
public:
    explicit MidLevelController(const MidLevelParams& p) : p_(p) {}

    MidLevelOutput compute(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                           const Eigen::Vector3d& ref_acc, double ref_psi,
                           const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                           const Eigen::Vector3d& euler);

private:
    MidLevelParams p_;
    double integral_x_ = 0.0, integral_y_ = 0.0, integral_z_ = 0.0;
};

// ---------------------------------------------------------------- Low level
struct LowLevelParams {
    double kp1 = 0, ki1 = 0, kd1 = 0;
    double kp2 = 0, ki2 = 0, kd2 = 0;
    double kp3 = 0, ki3 = 0, kd3 = 0;
    double integral_min = 0, integral_max = 0;
    double dt = 0.01;
};

class LowLevelController {
public:
    explicit LowLevelController(const LowLevelParams& p) : p_(p) {}

    // Returns reference_torques
    Eigen::Vector3d compute(const Eigen::Vector3d& ref_angles, const Eigen::Vector3d& euler);

private:
    LowLevelParams p_;
    Eigen::Vector3d prev_error_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
};

}  // namespace mqc_sim

#endif  // MQC_SIM_CONTROLLERS_H
