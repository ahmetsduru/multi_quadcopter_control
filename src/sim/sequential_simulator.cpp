#include "multi_quadcopter_control/sim/sequential_simulator.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace mqc_sim {

namespace {
Eigen::Vector3d toEigen(const Vec3& v) { return Eigen::Vector3d(v.x, v.y, v.z); }
}  // namespace

long long SequentialSimulator::divisor(double period, const std::string& what) const {
    if (!(period > 0.0)) {
        throw std::runtime_error(what + ": period must be > 0");
    }
    const double ratio = period / cfg_.dt;
    const long long n = std::llround(ratio);
    if (n < 1 || std::abs(ratio - static_cast<double>(n)) > 1e-6) {
        throw std::runtime_error(what + ": period (" + std::to_string(period) +
                                 " s) must be an integer multiple of dt (" + std::to_string(cfg_.dt) + " s)");
    }
    return n;
}

SequentialSimulator::SequentialSimulator(const SimConfig& config, const std::vector<DroneConfig>& drones)
    : cfg_(config) {
    if (!(cfg_.dt > 0.0)) throw std::runtime_error("dt must be > 0");

    downwash_.reset(new DownwashModel(cfg_.downwash));
    if (cfg_.disturbance_enabled) {
        dist_div_ = divisor(1.0 / cfg_.disturbance_rate, "disturbance_rate");
    }

    for (const DroneConfig& dc : drones) {
        if (std::abs(dc.quad.dt - cfg_.dt) > 1e-12) {
            throw std::runtime_error(dc.name + ": state_derivative_solver_node/dt differs from the simulation dt");
        }
        Drone d;
        d.cfg = dc;
        d.dyn.reset(new QuadDynamics(dc.quad));
        MidLevelParams mid = dc.mid;
        mid.ndob_tau_motor = dc.quad.tau_thrust;  // NDOB thrust model uses the actual actuator time constant
        d.mid.reset(new MidLevelController(mid));
        d.low.reset(new LowLevelController(dc.low));
        if (dc.use_se3) {
            Se3PositionParams sp = dc.se3_pos;
            sp.dt = dc.mid.dt;
            sp.mass = dc.quad.mass;
            Se3AttitudeParams sa = dc.se3_att;
            sa.dt = dc.low.dt;
            sa.J = dc.quad.I;
            d.se3_pos.reset(new Se3PositionController(sp));
            d.se3_att.reset(new Se3AttitudeController(sa));
            d.se3_out.thrust = dc.quad.mass * 9.81;
        }
        d.traj.reset(new TrajectorySequencer(dc.trajectories, dc.is_psi_active, dc.psi_filter_beta,
                                             dc.trajectory_time_scale));
        d.mid_div = divisor(dc.mid.dt, dc.name + " mid_level_controller/dt");
        d.low_div = divisor(dc.low.dt, dc.name + " low_level_controller/dt");
        d.next_traj_step = 0;
        downwash_->addDrone(dc.name, dc.quad.mass);
        drones_.push_back(std::move(d));
    }
}

bool SequentialSimulator::allTrajectoriesFinished() const {
    for (const Drone& d : drones_) {
        if (!d.traj->finished()) return false;
    }
    return true;
}

Eigen::Vector3d SequentialSimulator::referencePosition(size_t i) const {
    return toEigen(drones_[i].traj->position());
}

void SequentialSimulator::step() {
    // 1. Trajectory ticks
    for (Drone& d : drones_) {
        if (k_ >= d.next_traj_step && !d.traj->finished()) {
            d.traj->tick();
            double rate = d.traj->currentRate();
            long long div = std::llround(1.0 / (rate * cfg_.dt));
            if (div < 1) div = 1;
            d.next_traj_step += div;
        }
    }

    // 2. Downwash disturbance (held between updates). Uses only the state at t_k, so it can be
    //    computed before the controllers; the static part feeds the optional feedforward.
    if (cfg_.disturbance_enabled && k_ % dist_div_ == 0) {
        std::vector<DroneKinematics> kin(drones_.size());
        for (size_t i = 0; i < drones_.size(); ++i) {
            kin[i].position_world = drones_[i].dyn->position() + drones_[i].cfg.initial_offset;
            kin[i].R_body_to_world = drones_[i].dyn->rotation();
        }
        std::vector<Eigen::Vector3d> f_static;
        std::vector<Eigen::Vector3d> f = downwash_->compute(kin, &f_static);
        for (size_t i = 0; i < drones_.size(); ++i) {
            drones_[i].disturbance = f[i];
            drones_[i].disturbance_static = f_static[i];
        }
    }

    // 3. Mid-level controllers
    for (Drone& d : drones_) {
        if (k_ % d.mid_div == 0 && d.cfg.use_se3) {
            d.se3_out = d.se3_pos->compute(toEigen(d.traj->position()), toEigen(d.traj->velocity()),
                                           toEigen(d.traj->acceleration()), toEigen(d.traj->jerk()),
                                           d.traj->psi(), d.dyn->position(), d.dyn->velocity(),
                                           d.disturbance_static);
            d.mid_out.thrust = d.se3_out.thrust;
            d.mid_out.ref_angles = rotationToEuler(d.se3_out.R_d);  // for logging
            d.dyn->setThrustCommand(d.se3_out.thrust);
        } else if (k_ % d.mid_div == 0) {
            d.mid_out = d.mid->compute(toEigen(d.traj->position()), toEigen(d.traj->velocity()),
                                       toEigen(d.traj->acceleration()), d.traj->psi(),
                                       d.dyn->position(), d.dyn->velocity(), d.dyn->eulerAngles(),
                                       d.disturbance_static);
            d.dyn->setThrustCommand(d.mid_out.thrust);
        }
    }

    // 4. Low-level controllers
    for (Drone& d : drones_) {
        if (k_ % d.low_div == 0 && d.cfg.use_se3) {
            d.torques = d.se3_att->compute(d.se3_out.R_d, d.se3_out.omega_d, d.dyn->rotation(),
                                           d.dyn->angularVelocity());
            d.dyn->setTorqueCommand(d.torques);
        } else if (k_ % d.low_div == 0) {
            d.torques = d.low->compute(d.mid_out.ref_angles, d.dyn->eulerAngles());
            d.dyn->setTorqueCommand(d.torques);
        }
    }

    // 5. Log row at t_k
    const double t = time();
    for (Drone& d : drones_) {
        LogRow& r = d.last_row;
        r.time = t;
        r.a_pos = d.dyn->position() + d.cfg.initial_offset;
        r.a_vel = d.dyn->velocity();
        r.a_acc = d.dyn->acceleration(d.disturbance);
        r.a_ang_vel = d.dyn->angularVelocity();
        r.a_eul_ang = d.dyn->eulerAngles();
        r.r_pos = toEigen(d.traj->position()) + d.cfg.initial_offset;
        r.r_vel = toEigen(d.traj->velocity());
        r.r_acc = toEigen(d.traj->acceleration());
        r.r_eul_ang = d.mid_out.ref_angles;
        r.r_torq = d.torques;
        r.dist_f = d.disturbance;
        r.ndob_f = d.mid_out.ndob_estimate;
        r.r_thr = d.mid_out.thrust;
        r.r_psi = d.traj->psi();
    }

    // 6. Integrate dynamics t_k -> t_k + dt
    for (Drone& d : drones_) {
        d.dyn->step(d.disturbance);
    }
    ++k_;
}

}  // namespace mqc_sim
