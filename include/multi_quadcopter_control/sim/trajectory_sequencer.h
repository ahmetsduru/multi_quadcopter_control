// Trajectory sequencing (ROS-free).
// Same polynomial planners and switching logic as src/trajectory_generator.cpp and
// src/waypoint_server.cpp, but driven by an explicit tick() instead of ros::Rate loops.
#ifndef MQC_SIM_TRAJECTORY_SEQUENCER_H
#define MQC_SIM_TRAJECTORY_SEQUENCER_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mqc_sim {

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

struct TrajectorySpec {
    std::string name;
    std::vector<double> points_x, points_y, points_z, times;
    double ros_rate = 20.0;
    std::string method;  // cubic_spline, natural_cubic_spline, minimum_jerk, minimum_snap
};

class TrajectorySequencer {
public:
    // Local trajectory time advanced on every tick (hard-coded 0.05 s in trajectory_generator.cpp)
    static constexpr double kSampleStep = 0.05;

    // psi_filter_beta: low-pass coefficient of the trajectory-aligned yaw (Eq. 19), 0.05 in the original code
    TrajectorySequencer(const std::vector<TrajectorySpec>& specs, bool is_psi_active,
                        double psi_filter_beta = 0.05);

    // One iteration of the trajectory loop: evaluate the reference at the current local time,
    // switch to the next trajectory if needed, then advance the local time.
    void tick();

    bool finished() const { return finished_; }
    // Rate (Hz) of the trajectory currently being executed (ros_rate in the yaml)
    double currentRate() const { return current_.ros_rate; }
    const std::string& currentName() const { return current_.name; }

    // References (what the original node published)
    const Vec3& position() const { return m_position; }
    const Vec3& velocity() const { return m_velocity; }
    const Vec3& acceleration() const { return m_acceleration; }
    const Vec3& jerk() const { return m_jerk; }
    double psi() const { return m_des_psi; }

private:
    // Trajectory "server"
    std::vector<TrajectorySpec> specs_;
    size_t next_index_ = 0;
    bool m_no_more_trajectory = false;
    bool fetchNext();  // = getTrajectoryFromServer()

    // Current trajectory and its plan
    TrajectorySpec current_;
    bool have_current_ = false;
    bool active_ = false;  // plan solved and being sampled
    bool finished_ = false;
    int n_ = 0;             // number of segments
    int coeffs_per_seg_ = 4;
    int switch_segment_ = 5;
    bool updates_psi_ = true;  // solveNaturalCubicSpline() never calls generatePsi()
    std::vector<double> times_mod_;
    Eigen::VectorXd cx_, cy_, cz_;
    double t_ = 0.0;
    bool plan();  // = solveXxx() set-up, returns false for an unknown method
    int evaluate(double t);

    // Planner state (same names as in trajectory_generator.cpp)
    bool m_initial_trajectory = true;
    bool m_is_psi_active = true;
    double psi_filter_beta_ = 0.05;
    Vec3 m_position, m_velocity, m_acceleration, m_jerk;
    double m_des_psi = 0.0;
    double m_last_psi = 0.0;
    void generatePsi();

    void buildNaturalCubicSpline(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                                 const std::vector<double>& points_z_mod, const std::vector<double>& times_mod);
    void buildCubicSpline(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                          const std::vector<double>& points_z_mod, const std::vector<double>& times_mod);
    void buildMinimumJerk(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                          const std::vector<double>& points_z_mod, const std::vector<double>& times_mod);
    void buildMinimumSnap(const std::vector<double>& points_x_mod, const std::vector<double>& points_y_mod,
                          const std::vector<double>& points_z_mod, const std::vector<double>& times_mod);
};

}  // namespace mqc_sim

#endif  // MQC_SIM_TRAJECTORY_SEQUENCER_H
