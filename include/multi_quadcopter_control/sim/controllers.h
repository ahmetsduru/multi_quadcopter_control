// Mid-level (position) and low-level (attitude) controllers (ROS-free).
// Same control laws as src/mid_level_cont.cpp and src/low_level_cont.cpp.
// The mid-level controller is the hierarchical PID of src/mid_level_cont.cpp in every mode; the modes
// differ only in the disturbance compensation term subtracted from the desired force:
//   PID                              : none
//   PID + disturbance_feedforward    : modelled downwash f_dw (Eq. 21)
//   NDOB                             : disturbance delta_f estimated by a nonlinear disturbance observer
#ifndef MQC_SIM_CONTROLLERS_H
#define MQC_SIM_CONTROLLERS_H

#include <Eigen/Dense>

namespace mqc_sim {

// ---------------------------------------------------------------- Mid level
enum class MidLevelMode { PID, NDOB };

struct MidLevelParams {
    MidLevelMode mode = MidLevelMode::PID;
    double kp_x = 0, ki_x = 0, kd_x = 0;
    double kp_y = 0, ki_y = 0, kd_y = 0;
    double kp_z = 0, ki_z = 0, kd_z = 0;
    double integral_min = 0, integral_max = 0;
    double min_thrust = 0, max_thrust = 0;
    double dt = 0.05;
    double mass = 0.382;
    bool disturbance_feedforward = false;  // f_des -= f_dw (modelled downwash, Eq. 21)

    // NDOB (mid_level_controller/lv_x, lv_y, lv_z): observer bandwidths (rad/s)
    double lv_x = 2.0, lv_y = 2.0, lv_z = 2.5;
    double ndob_tau_motor = 0.1;   // observer thrust filter time constant (s); the simulator sets it to tau_thrust
};

struct MidLevelOutput {
    double thrust = 0.0;                                     // reference_thrust
    Eigen::Vector3d ref_angles = Eigen::Vector3d::Zero();    // reference_euler_angles
    Eigen::Vector3d ndob_estimate = Eigen::Vector3d::Zero(); // NDOB: estimated disturbance (delta_f)
};

class MidLevelController {
public:
    explicit MidLevelController(const MidLevelParams& p)
        : p_(p), last_thrust_(p.mass * 9.81), filtered_thrust_(p.mass * 9.81) {}

    // f_dw: modelled downwash force on this drone (world frame), used only if
    // disturbance_feedforward is enabled.
    MidLevelOutput compute(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                           const Eigen::Vector3d& ref_acc, double ref_psi,
                           const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                           const Eigen::Vector3d& euler,
                           const Eigen::Vector3d& f_dw = Eigen::Vector3d::Zero());

private:
    MidLevelParams p_;
    double integral_x_ = 0.0, integral_y_ = 0.0, integral_z_ = 0.0;

    // NDOB state
    Eigen::Vector3d zv_ = Eigen::Vector3d::Zero();       // observer internal state
    Eigen::Vector3d delta_f_ = Eigen::Vector3d::Zero();  // estimated disturbance
    double last_thrust_;                                 // last commanded thrust (starts at hover)
    double filtered_thrust_;                             // first-order filtered thrust (starts at hover)

    MidLevelOutput computeLaw(const Eigen::Vector3d& ref_pos, const Eigen::Vector3d& ref_vel,
                              const Eigen::Vector3d& ref_acc, double ref_psi,
                              const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                              const Eigen::Vector3d& euler, const Eigen::Vector3d& f_comp, bool use_comp);
    void updateObserver(const Eigen::Vector3d& vel, const Eigen::Vector3d& euler);
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
