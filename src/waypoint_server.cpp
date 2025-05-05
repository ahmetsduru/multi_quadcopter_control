#include "../include/multi_quadcopter_control/waypoint_server.h"

// Constructor: Initialize the server and load parameters
WaypointServer::WaypointServer(ros::NodeHandle& nh) : m_current_trajectory_index(0), m_last_time_offset(0.0) {
    // Create the service server
    m_service = nh.advertiseService("get_trajectory", &WaypointServer::getTrajectoryCallback, this);

    // Read the active trajectories parameter
    if (nh.getParam("trajectory_manager/active_trajectories", m_active_trajectories)) {
        ROS_ASSERT(m_active_trajectories.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int i = 0; i < m_active_trajectories.size(); ++i) {
            std::string trajectory_name = static_cast<std::string>(m_active_trajectories[i]);
            loadTrajectory(nh, trajectory_name);  // Load each active trajectory
        }
    } else {
        ROS_ERROR("Failed to get active_trajectories from trajectory_manager");
    }
}

// Callback function for the service
bool WaypointServer::getTrajectoryCallback(multi_quadcopter_control::WaypointService::Request &req,
                                           multi_quadcopter_control::WaypointService::Response &res) {
    static bool all_trajectories_logged = false;  // Flag to prevent repeated message logging

    if (m_current_trajectory_index >= m_trajectories.size()) {
        res.points_x.clear();
        res.points_y.clear();
        res.points_z.clear();
        res.times.clear();

        if (!all_trajectories_logged) {  // Log the message only once
            ROS_INFO("All trajectories completed. Sending empty trajectory points and times.");
            all_trajectories_logged = true;  // Mark message as logged to prevent repetition
        }

        return true;
    }

    all_trajectories_logged = false;  // Reset flag if there are more trajectories to send

    const auto& traj = m_trajectories[m_current_trajectory_index];

    // Send the next trajectory data in the response without applying any offset
    res.method = traj.m_method;
    res.ros_rate = traj.m_ros_rate;
    res.points_x = traj.m_points_x;
    res.points_y = traj.m_points_y;
    res.points_z = traj.m_points_z;
    res.times = traj.m_times;  // Directly assign without offset

    m_current_trajectory_index++;

    return true;
}

void WaypointServer::loadTrajectory(ros::NodeHandle& nh, const std::string& trajectory_name) {
    TrajectoryData traj;
    std::string base_path = "/trajectories/" + trajectory_name;

    nh.getParam(base_path + "/points_x", traj.m_points_x);
    nh.getParam(base_path + "/points_y", traj.m_points_y);
    nh.getParam(base_path + "/points_z", traj.m_points_z);
    nh.getParam(base_path + "/times", traj.m_times);
    nh.getParam(base_path + "/ros_rate", traj.m_ros_rate);
    nh.getParam(base_path + "/method", traj.m_method);

    m_trajectories.push_back(traj);
    ROS_INFO("Loaded trajectory: %s", trajectory_name.c_str());
}

