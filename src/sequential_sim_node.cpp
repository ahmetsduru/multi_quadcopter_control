// Deterministic sequential multi-quadcopter simulation node.
//
// Replaces waypoint_server + trajectory_generator + mid_level_cont + low_level_cont + eom
// + relative_disturbance_generator + data_logger + rviz_data_handler with a single loop that
// runs every component in a fixed order with a step counter as the clock (see
// sim/sequential_simulator.h). Runs with the same yaml parameters, writes log/<drone>_log.txt
// in the same format, and publishes TF / paths for RViz.
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "multi_quadcopter_control/sim/sequential_simulator.h"

using namespace mqc_sim;

namespace {

template <typename T>
T requireParam(const ros::NodeHandle& nh, const std::string& key) {
    T value;
    if (!nh.getParam(key, value)) {
        throw std::runtime_error("Missing parameter: " + key);
    }
    return value;
}

Eigen::Matrix3d matrixParam(const ros::NodeHandle& nh, const std::string& key) {
    std::vector<double> v = requireParam<std::vector<double>>(nh, key);
    if (v.size() != 9) throw std::runtime_error(key + " must have 9 elements");
    Eigen::Matrix3d m;
    for (int i = 0; i < 9; ++i) m(i / 3, i % 3) = v[i];
    return m;
}

DroneConfig loadDrone(const ros::NodeHandle& nh, const std::string& name) {
    const std::string ns = "/" + name + "/";
    DroneConfig d;
    d.name = name;

    double ox = 0.0, oy = 0.0, oz = 0.0;
    nh.param(ns + "initial_x", ox, 0.0);
    nh.param(ns + "initial_y", oy, 0.0);
    nh.param(ns + "initial_z", oz, 0.0);
    d.initial_offset = Eigen::Vector3d(ox, oy, oz);

    // state_derivative_solver_node (eom)
    const std::string q = ns + "state_derivative_solver_node/";
    d.quad.mass = requireParam<double>(nh, q + "mass");
    d.quad.dt = requireParam<double>(nh, q + "dt");
    d.quad.tau_thrust = requireParam<double>(nh, q + "tau_thrust");
    d.quad.tau_torque = requireParam<double>(nh, q + "tau_torque");
    d.quad.rho = requireParam<double>(nh, q + "rho");
    d.quad.I = matrixParam(nh, q + "I");
    d.quad.Cd_trans = matrixParam(nh, q + "Cd_trans");
    d.quad.A_trans = matrixParam(nh, q + "A_trans");
    std::vector<double> g = requireParam<std::vector<double>>(nh, q + "g");
    if (g.size() != 3) throw std::runtime_error(q + "g must have 3 elements");
    d.quad.g = Eigen::Vector3d(g[0], g[1], g[2]);

    // mid_level_controller
    const std::string m = ns + "mid_level_controller/";
    d.mid.kp_x = requireParam<double>(nh, m + "kp_thrust_x");
    d.mid.ki_x = requireParam<double>(nh, m + "ki_thrust_x");
    d.mid.kd_x = requireParam<double>(nh, m + "kd_thrust_x");
    d.mid.kp_y = requireParam<double>(nh, m + "kp_thrust_y");
    d.mid.ki_y = requireParam<double>(nh, m + "ki_thrust_y");
    d.mid.kd_y = requireParam<double>(nh, m + "kd_thrust_y");
    d.mid.kp_z = requireParam<double>(nh, m + "kp_thrust_z");
    d.mid.ki_z = requireParam<double>(nh, m + "ki_thrust_z");
    d.mid.kd_z = requireParam<double>(nh, m + "kd_thrust_z");
    d.mid.integral_min = requireParam<double>(nh, m + "integral_min");
    d.mid.integral_max = requireParam<double>(nh, m + "integral_max");
    d.mid.min_thrust = requireParam<double>(nh, m + "min_thrust");
    d.mid.max_thrust = requireParam<double>(nh, m + "max_thrust");
    d.mid.dt = requireParam<double>(nh, m + "dt");
    d.mid.mass = d.quad.mass;
    // NDOB observer bandwidths (used only with controller_type = ndob)
    nh.param(m + "lv_x", d.mid.lv_x, 2.0);
    nh.param(m + "lv_y", d.mid.lv_y, 2.0);
    nh.param(m + "lv_z", d.mid.lv_z, 2.5);

    // low_level_controller
    const std::string l = ns + "low_level_controller/";
    d.low.kp1 = requireParam<double>(nh, l + "kp_torque1");
    d.low.ki1 = requireParam<double>(nh, l + "ki_torque1");
    d.low.kd1 = requireParam<double>(nh, l + "kd_torque1");
    d.low.kp2 = requireParam<double>(nh, l + "kp_torque2");
    d.low.ki2 = requireParam<double>(nh, l + "ki_torque2");
    d.low.kd2 = requireParam<double>(nh, l + "kd_torque2");
    d.low.kp3 = requireParam<double>(nh, l + "kp_torque3");
    d.low.ki3 = requireParam<double>(nh, l + "ki_torque3");
    d.low.kd3 = requireParam<double>(nh, l + "kd_torque3");
    d.low.integral_min = requireParam<double>(nh, l + "integral_min");
    d.low.integral_max = requireParam<double>(nh, l + "integral_max");
    d.low.dt = requireParam<double>(nh, l + "dt");

    // trajectory_manager + /trajectories/<name>
    nh.param(ns + "trajectory_manager/is_psi_active", d.is_psi_active, true);
    std::vector<std::string> active =
        requireParam<std::vector<std::string>>(nh, ns + "trajectory_manager/active_trajectories");
    for (const std::string& tn : active) {
        const std::string base = "/trajectories/" + tn + "/";
        TrajectorySpec s;
        s.name = tn;
        s.points_x = requireParam<std::vector<double>>(nh, base + "points_x");
        s.points_y = requireParam<std::vector<double>>(nh, base + "points_y");
        s.points_z = requireParam<std::vector<double>>(nh, base + "points_z");
        s.times = requireParam<std::vector<double>>(nh, base + "times");
        s.ros_rate = requireParam<double>(nh, base + "ros_rate");
        s.method = requireParam<std::string>(nh, base + "method");
        if (std::abs(1.0 / s.ros_rate - TrajectorySequencer::kSampleStep) > 1e-9) {
            ROS_WARN("%s/%s: ros_rate=%.1f Hz but the trajectory time step is %.3f s "
                     "(same behaviour as trajectory_generator: the reference runs %.2fx real time)",
                     name.c_str(), tn.c_str(), s.ros_rate, TrajectorySequencer::kSampleStep,
                     TrajectorySequencer::kSampleStep * s.ros_rate);
        }
        d.trajectories.push_back(s);
    }
    ROS_INFO("Sequential sim: %s loaded (mass %.3f kg, %zu trajectories, offset %.2f %.2f %.2f)",
             name.c_str(), d.quad.mass, d.trajectories.size(), ox, oy, oz);
    return d;
}

geometry_msgs::Vector3 toMsg(const Eigen::Vector3d& v) {
    geometry_msgs::Vector3 m;
    m.x = v.x();
    m.y = v.y();
    m.z = v.z();
    return m;
}

struct DroneRos {
    ros::Publisher path_pub, ref_path_pub;
    ros::Publisher pos_pub, euler_pub, ref_pos_pub, dist_pub, thrust_pub, ndob_pub;
    nav_msgs::Path path, ref_path;
    std::unique_ptr<DroneLogWriter> log;
    double sq_err_sum = 0.0, max_err = 0.0;
    long long samples = 0;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "sequential_sim");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ---------------- Simulation options
    double real_time_factor, sim_duration, end_hold_time, viz_rate, path_rate, disturbance_rate;
    bool log_enabled, disturbance_enabled, publish_topics, shutdown_on_finish, disturbance_feedforward;
    double psi_filter_beta;
    std::string controller_type;
    int log_decimation, noise_seed;
    std::string log_dir;
    pnh.param("real_time_factor", real_time_factor, 1.0);  // <= 0: as fast as possible
    pnh.param("sim_duration", sim_duration, -1.0);         // <= 0: until all trajectories end + end_hold_time
    pnh.param("end_hold_time", end_hold_time, 2.0);
    pnh.param("viz_rate", viz_rate, 30.0);    // TF + topics (Hz, simulation time)
    pnh.param("path_rate", path_rate, 10.0);  // RViz paths (Hz, simulation time)
    pnh.param("publish_topics", publish_topics, true);
    pnh.param("shutdown_on_finish", shutdown_on_finish, false);
    pnh.param("log_enabled", log_enabled, true);
    pnh.param("log_decimation", log_decimation, 1);
    pnh.param("log_dir", log_dir, ros::package::getPath("multi_quadcopter_control") + "/log");

    SimConfig cfg;
    pnh.param("disturbance_enabled", disturbance_enabled, true);
    pnh.param("disturbance_rate", disturbance_rate, 500.0);
    pnh.param("noise_seed", noise_seed, 42);
    pnh.param("c_coup", cfg.downwash.c_coup, 0.09);
    pnh.param("r_lat", cfg.downwash.r_lat, 0.05);
    pnh.param("z0", cfg.downwash.z0, std::sqrt(0.05));
    pnh.param("sigma", cfg.downwash.sigma, 0.15);
    pnh.param("turb_intensity", cfg.downwash.turb_intensity, 0.70);
    pnh.param("max_vertical_range", cfg.downwash.max_vertical_range, -1.0);
    cfg.downwash.noise_seed = noise_seed;
    cfg.disturbance_enabled = disturbance_enabled;
    cfg.disturbance_rate = disturbance_rate;
    // Controller / trajectory options (same for every drone)
    pnh.param("psi_filter_beta", psi_filter_beta, 0.05);                   // yaw low-pass beta (Eq. 19)
    // Mid-level controller:
    //   pid_ff : hierarchical PID + modelled-downwash feedforward f_des -= f_dw (Eq. 21)  [default]
    //   pid    : hierarchical PID without feedforward (ablation)
    //   ndob   : PID + nonlinear disturbance observer (no model feedforward; the observer
    //            estimates the disturbance, enabling both would compensate it twice)
    pnh.param("controller_type", controller_type, std::string("pid_ff"));
    MidLevelMode mid_mode;
    if (controller_type == "pid_ff") {
        mid_mode = MidLevelMode::PID;
        disturbance_feedforward = true;
    } else if (controller_type == "pid") {
        mid_mode = MidLevelMode::PID;
        disturbance_feedforward = false;
    } else if (controller_type == "ndob") {
        mid_mode = MidLevelMode::NDOB;
        disturbance_feedforward = false;
    } else {
        ROS_FATAL("Sequential sim: unknown controller_type '%s' (use 'pid_ff', 'pid' or 'ndob')",
                  controller_type.c_str());
        return 1;
    }
    const bool ndob_mode = (mid_mode == MidLevelMode::NDOB);
    if (log_decimation < 1) log_decimation = 1;

    // ---------------- Drones
    int num_drones = 1;
    nh.param("/num_drones", num_drones, 1);

    std::unique_ptr<SequentialSimulator> sim;
    try {
        std::vector<DroneConfig> drones;
        for (int i = 1; i <= num_drones; ++i) {
            DroneConfig dc = loadDrone(nh, "drone" + std::to_string(i));
            dc.mid.disturbance_feedforward = disturbance_feedforward;
            dc.mid.mode = mid_mode;
            if (ndob_mode) {
                ROS_INFO("Sequential sim: %s NDOB gains lv = [%.3f %.3f %.3f]", dc.name.c_str(),
                         dc.mid.lv_x, dc.mid.lv_y, dc.mid.lv_z);
            }
            dc.psi_filter_beta = psi_filter_beta;
            drones.push_back(dc);
        }
        if (drones.empty()) throw std::runtime_error("num_drones must be >= 1");
        cfg.dt = drones.front().quad.dt;
        sim.reset(new SequentialSimulator(cfg, drones));
    } catch (const std::exception& e) {
        ROS_FATAL("Sequential sim: %s", e.what());
        return 1;
    }

    ROS_INFO("Sequential sim: controller '%s' (%s), psi_filter_beta=%.3f", controller_type.c_str(),
             ndob_mode ? "PID + NDOB" : (disturbance_feedforward ? "PID + downwash feedforward" : "PID, no feedforward"),
             psi_filter_beta);
    ROS_INFO("Sequential sim: %d drones, dt=%.4f s, disturbance %s (%.0f Hz, noise_seed=%d, z0=%.4f), "
             "real_time_factor=%.2f",
             num_drones, cfg.dt, disturbance_enabled ? "on" : "off", disturbance_rate, noise_seed,
             cfg.downwash.z0, real_time_factor);

    // ---------------- ROS outputs
    tf2_ros::TransformBroadcaster tf_broadcaster;
    std::vector<DroneRos> out(sim->droneCount());
    if (log_enabled) {
        std::string cmd = "mkdir -p \"" + log_dir + "\"";
        int ret = system(cmd.c_str());
        (void)ret;
    }
    for (size_t i = 0; i < sim->droneCount(); ++i) {
        const std::string ns = "/" + sim->droneName(i) + "/";
        out[i].path_pub = nh.advertise<nav_msgs::Path>(ns + "actual_position_path", 1);
        out[i].ref_path_pub = nh.advertise<nav_msgs::Path>(ns + "reference_position_path", 1);
        out[i].path.header.frame_id = "world";
        out[i].ref_path.header.frame_id = "world";
        if (publish_topics) {
            out[i].pos_pub = nh.advertise<geometry_msgs::Vector3>(ns + "actual_position", 10);
            out[i].euler_pub = nh.advertise<geometry_msgs::Vector3>(ns + "actual_euler_angles", 10);
            out[i].ref_pos_pub = nh.advertise<geometry_msgs::Vector3>(ns + "reference_position", 10);
            out[i].dist_pub = nh.advertise<geometry_msgs::Vector3>(ns + "disturbance_force", 10);
            out[i].thrust_pub = nh.advertise<std_msgs::Float64>(ns + "reference_thrust", 10);
            if (ndob_mode) {
                out[i].ndob_pub = nh.advertise<geometry_msgs::Vector3>(ns + "ndob_estimated_disturbance", 10);
            }
        }
        if (log_enabled) {
            const std::string path = log_dir + "/" + sim->droneName(i) + "_log.txt";
            out[i].log.reset(new DroneLogWriter(path, ndob_mode));
            if (!out[i].log->isOpen()) {
                ROS_ERROR("Sequential sim: cannot open %s", path.c_str());
            } else {
                ROS_INFO("Sequential sim: logging %s to %s", sim->droneName(i).c_str(), path.c_str());
            }
        }
    }

    const long long viz_div = std::max<long long>(1, std::llround(1.0 / (viz_rate * cfg.dt)));
    const long long path_div = std::max<long long>(1, std::llround(1.0 / (path_rate * cfg.dt)));

    auto publishViz = [&](bool publish_state, bool append_path) {
        const ros::Time stamp = ros::Time::now();
        for (size_t i = 0; i < sim->droneCount(); ++i) {
            const QuadDynamics& dyn = sim->dynamics(i);
            const LogRow& row = sim->lastLogRow(i);
            const Eigen::Vector3d p = dyn.position() + sim->initialOffset(i);
            const Eigen::Quaterniond q(dyn.rotation());

            geometry_msgs::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = "world";
            tf.child_frame_id = sim->droneName(i) + "/base_link";
            tf.transform.translation.x = p.x();
            tf.transform.translation.y = p.y();
            tf.transform.translation.z = p.z();
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf_broadcaster.sendTransform(tf);

            if (publish_state && publish_topics) {
                // Same frames as the original nodes (positions relative to the initial offset)
                out[i].pos_pub.publish(toMsg(dyn.position()));
                out[i].euler_pub.publish(toMsg(dyn.eulerAngles()));
                out[i].ref_pos_pub.publish(toMsg(sim->referencePosition(i)));
                out[i].dist_pub.publish(toMsg(row.dist_f));
                std_msgs::Float64 thr;
                thr.data = row.r_thr;
                out[i].thrust_pub.publish(thr);
                if (ndob_mode) out[i].ndob_pub.publish(toMsg(row.ndob_f));
            }

            if (!append_path) continue;

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = stamp;
            pose.header.frame_id = "world";
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();
            pose.pose.orientation = tf.transform.rotation;
            out[i].path.poses.push_back(pose);
            out[i].path.header.stamp = stamp;
            out[i].path_pub.publish(out[i].path);

            const Eigen::Vector3d r = sim->referencePosition(i) + sim->initialOffset(i);
            geometry_msgs::PoseStamped ref_pose;
            ref_pose.header = pose.header;
            ref_pose.pose.position.x = r.x();
            ref_pose.pose.position.y = r.y();
            ref_pose.pose.position.z = r.z();
            ref_pose.pose.orientation.w = 1.0;
            out[i].ref_path.poses.push_back(ref_pose);
            out[i].ref_path.header.stamp = stamp;
            out[i].ref_path_pub.publish(out[i].ref_path);
        }
    };

    // ---------------- Main loop
    const auto wall_start = std::chrono::steady_clock::now();
    double t_traj_finished = -1.0;
    bool done = false;

    while (ros::ok() && !done) {
        sim->step();
        const long long k = sim->stepCount() - 1;  // index of the step just executed
        const double t_next = sim->time();

        for (size_t i = 0; i < sim->droneCount(); ++i) {
            const LogRow& row = sim->lastLogRow(i);
            if (out[i].log && (k % log_decimation == 0)) out[i].log->write(row);
            const double e = (row.r_pos - row.a_pos).norm();
            out[i].sq_err_sum += e * e;
            out[i].max_err = std::max(out[i].max_err, e);
            out[i].samples++;
        }

        const bool viz_tick = (k % viz_div == 0);
        const bool path_tick = (k % path_div == 0);
        if (viz_tick || path_tick) {
            publishViz(viz_tick, path_tick);
            ros::spinOnce();
        }

        if (t_traj_finished < 0.0 && sim->allTrajectoriesFinished()) {
            t_traj_finished = t_next;
            ROS_INFO("Sequential sim: all trajectories finished at t=%.3f s", t_traj_finished);
        }
        if (sim_duration > 0.0) {
            done = t_next >= sim_duration - 1e-9;
        } else {
            done = t_traj_finished >= 0.0 && t_next >= t_traj_finished + end_hold_time - 1e-9;
        }

        // Real-time pacing (does not change the results, only how fast they are produced)
        if (real_time_factor > 0.0 && (k % viz_div == 0)) {
            const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                 std::chrono::duration<double>(t_next / real_time_factor));
            std::this_thread::sleep_until(target);
        }
    }

    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    for (size_t i = 0; i < sim->droneCount(); ++i) {
        if (out[i].log) out[i].log->close();
        const double rms = out[i].samples ? std::sqrt(out[i].sq_err_sum / out[i].samples) : 0.0;
        ROS_INFO("Sequential sim: %s position error RMS=%.4f m, max=%.4f m", sim->droneName(i).c_str(), rms,
                 out[i].max_err);
    }
    ROS_INFO("Sequential sim: finished at t=%.3f s (%lld steps) in %.2f s wall time", sim->time(),
             sim->stepCount(), wall);

    if (shutdown_on_finish) {
        ros::shutdown();
        return 0;
    }

    // Keep the final state visible in RViz
    ros::Rate idle(10.0);
    while (ros::ok()) {
        publishViz(false, false);
        ros::spinOnce();
        idle.sleep();
    }
    return 0;
}
