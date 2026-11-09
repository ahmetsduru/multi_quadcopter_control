// Deterministic, single-threaded multi-quadcopter simulation (ROS-free).
//
// Every time step dt (= eom dt) is executed in a fixed order, like a MATLAB loop:
//   1. trajectory tick      (every 1/ros_rate s)
//   2. downwash disturbance (every 1/disturbance_rate s; depends only on the state at t_k)
//   3. mid-level controller (every mid_level_controller/dt s)
//   4. low-level controller (every low_level_controller/dt s)
//   5. log row for time t_k (state at t_k, commands computed at t_k)
//   6. dynamics integration t_k -> t_k + dt
// Time is a step counter (t_k = k * dt), so results do not depend on CPU load or wall clock.
#ifndef MQC_SIM_SEQUENTIAL_SIMULATOR_H
#define MQC_SIM_SEQUENTIAL_SIMULATOR_H

#include <memory>
#include <string>
#include <vector>

#include "multi_quadcopter_control/sim/controllers.h"
#include "multi_quadcopter_control/sim/downwash_model.h"
#include "multi_quadcopter_control/sim/drone_log_writer.h"
#include "multi_quadcopter_control/sim/quad_dynamics.h"
#include "multi_quadcopter_control/sim/se3_controller.h"
#include "multi_quadcopter_control/sim/trajectory_sequencer.h"

namespace mqc_sim {

struct DroneConfig {
    std::string name;
    Eigen::Vector3d initial_offset = Eigen::Vector3d::Zero();
    QuadParams quad;
    MidLevelParams mid;
    LowLevelParams low;
    std::vector<TrajectorySpec> trajectories;
    bool is_psi_active = true;
    double psi_filter_beta = 0.05;

    // Geometric SE(3) controller instead of the PID family (mid/low params then only give the rates)
    bool use_se3 = false;
    Se3PositionParams se3_pos;
    Se3AttitudeParams se3_att;
};

struct SimConfig {
    double dt = 0.002;               // integration step (eom dt)
    bool disturbance_enabled = true;
    double disturbance_rate = 500.0; // Hz
    DownwashParams downwash;
};

class SequentialSimulator {
public:
    // Throws std::runtime_error if controller periods are not integer multiples of dt.
    SequentialSimulator(const SimConfig& config, const std::vector<DroneConfig>& drones);

    void step();  // advance by one dt

    long long stepCount() const { return k_; }
    double time() const { return static_cast<double>(k_) * cfg_.dt; }
    double dt() const { return cfg_.dt; }
    bool allTrajectoriesFinished() const;

    size_t droneCount() const { return drones_.size(); }
    const std::string& droneName(size_t i) const { return drones_[i].cfg.name; }
    const Eigen::Vector3d& initialOffset(size_t i) const { return drones_[i].cfg.initial_offset; }
    const QuadDynamics& dynamics(size_t i) const { return *drones_[i].dyn; }
    Eigen::Vector3d referencePosition(size_t i) const;
    // Log row of the last executed step (time t_k, positions include the initial offset)
    const LogRow& lastLogRow(size_t i) const { return drones_[i].last_row; }

private:
    struct Drone {
        DroneConfig cfg;
        std::unique_ptr<QuadDynamics> dyn;
        std::unique_ptr<MidLevelController> mid;
        std::unique_ptr<LowLevelController> low;
        std::unique_ptr<Se3PositionController> se3_pos;
        std::unique_ptr<Se3AttitudeController> se3_att;
        Se3PositionOutput se3_out;
        std::unique_ptr<TrajectorySequencer> traj;
        long long mid_div = 1, low_div = 1;
        long long next_traj_step = 0;

        MidLevelOutput mid_out;
        Eigen::Vector3d torques = Eigen::Vector3d::Zero();
        Eigen::Vector3d disturbance = Eigen::Vector3d::Zero();
        Eigen::Vector3d disturbance_static = Eigen::Vector3d::Zero();  // modelled downwash (feedforward)
        LogRow last_row;
    };

    SimConfig cfg_;
    std::vector<Drone> drones_;
    std::unique_ptr<DownwashModel> downwash_;
    long long dist_div_ = 1;
    long long k_ = 0;

    long long divisor(double period, const std::string& what) const;
};

}  // namespace mqc_sim

#endif  // MQC_SIM_SEQUENTIAL_SIMULATOR_H
