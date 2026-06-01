#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>
#include <ros/package.h>

struct DroneData {
    geometry_msgs::Vector3 actual_position, actual_velocity, actual_acceleration;
    geometry_msgs::Vector3 actual_angular_velocity, actual_euler_angles;
    geometry_msgs::Vector3 reference_position, reference_velocity, reference_acceleration;
    geometry_msgs::Vector3 reference_euler_angles, reference_torques;
    geometry_msgs::Vector3 disturbance_force;
    geometry_msgs::Vector3 ndob_estimated_disturbance; 
    double reference_thrust = 0.0;
    double reference_psi = 0.0;
};

class DroneLogger {
public:
    DroneLogger(const std::string& drone_name)
        : m_drone_name(drone_name)
    {
        std::string ns = "/" + m_drone_name + "/";

        // Offset değerlerini al
        m_nh.param(ns + "initial_x", m_initial_offset.x, 0.0);
        m_nh.param(ns + "initial_y", m_initial_offset.y, 0.0);
        m_nh.param(ns + "initial_z", m_initial_offset.z, 0.0);

        subscribeVector(ns + "actual_position", &DroneData::actual_position, true);
        subscribeVector(ns + "actual_velocity", &DroneData::actual_velocity);
        subscribeVector(ns + "actual_acceleration", &DroneData::actual_acceleration);
        subscribeVector(ns + "actual_angular_velocity", &DroneData::actual_angular_velocity);
        subscribeVector(ns + "actual_euler_angles", &DroneData::actual_euler_angles);

        subscribeVector(ns + "reference_position", &DroneData::reference_position, true);
        subscribeVector(ns + "reference_velocity", &DroneData::reference_velocity);
        subscribeVector(ns + "reference_acceleration", &DroneData::reference_acceleration);
        subscribeVector(ns + "reference_euler_angles", &DroneData::reference_euler_angles);
        subscribeVector(ns + "reference_torques", &DroneData::reference_torques);
        subscribeVector(ns + "disturbance_force", &DroneData::disturbance_force);
        subscribeVector(ns + "ndob_estimated_disturbance", &DroneData::ndob_estimated_disturbance);

        subscribeFloat(ns + "reference_thrust", &DroneData::reference_thrust);
        subscribeFloat(ns + "reference_psi", &DroneData::reference_psi);

        std::string log_folder = ros::package::getPath("multi_quadcopter_control") + "/log/";
        std::string cmd = "mkdir -p " + log_folder;
        int ret = system(cmd.c_str()); (void)ret;

        std::string log_file_path = log_folder + m_drone_name + "_log.txt";
        m_log_file.open(log_file_path, std::ios::out);
        writeHeader();

        // 0.5 saniyelik gecikmeden sonra timer başlat
        std::thread([this]() {
            ros::Duration(0.35).sleep();
            m_timer = m_nh.createTimer(ros::Duration(1.0 / 500.0), &DroneLogger::timerCallback, this);
        }).detach();
    }

    ~DroneLogger() {
        if (m_log_file.is_open()) {
            m_log_file.close();
        }
    }

private:
    std::string m_drone_name;
    std::ofstream m_log_file;
    ros::NodeHandle m_nh;
    ros::Timer m_timer;
    DroneData m_data;
    geometry_msgs::Vector3 m_initial_offset;
    std::vector<ros::Subscriber> m_subscribers;

    void writeHeader() {
        m_log_file << "time";
        for (const auto& name : {
            "a_pos", "a_vel", "a_acc",
            "a_ang_vel", "a_eul_ang",
            "r_pos", "r_vel", "r_acc",
            "r_eul_ang", "r_torq", "dist_f", "ndob_f"
        }) {
            m_log_file << ", " << name << ".x"
                       << ", " << name << ".y"
                       << ", " << name << ".z";
        }
        m_log_file << ", r_thr"
                   << ", r_psi" << "\n";
    }

    void writeDataRow() {
        double now = ros::Time::now().toSec();

        m_log_file << std::fixed << std::setprecision(9) << now;
        m_log_file.unsetf(std::ios::floatfield);

        auto writeVec = [&](const geometry_msgs::Vector3& vec) {
            m_log_file << ", " << vec.x
                       << ", " << vec.y
                       << ", " << vec.z;
        };

        writeVec(m_data.actual_position);
        writeVec(m_data.actual_velocity);
        writeVec(m_data.actual_acceleration);
        writeVec(m_data.actual_angular_velocity);
        writeVec(m_data.actual_euler_angles);
        writeVec(m_data.reference_position);
        writeVec(m_data.reference_velocity);
        writeVec(m_data.reference_acceleration);
        writeVec(m_data.reference_euler_angles);
        writeVec(m_data.reference_torques);
        writeVec(m_data.disturbance_force);
        writeVec(m_data.ndob_estimated_disturbance);

        m_log_file << ", " << m_data.reference_thrust
                   << ", " << m_data.reference_psi << "\n";
    }

    void timerCallback(const ros::TimerEvent&) {
        writeDataRow();
    }

    void subscribeVector(const std::string& topic, geometry_msgs::Vector3 DroneData::*field, bool apply_offset = false) {
        ros::Subscriber sub = m_nh.subscribe<geometry_msgs::Vector3>(topic, 10,
            [this, field, apply_offset](const geometry_msgs::Vector3::ConstPtr& msg) {
                geometry_msgs::Vector3 temp = *msg;
                if (apply_offset) {
                    temp.x += m_initial_offset.x;
                    temp.y += m_initial_offset.y;
                    temp.z += m_initial_offset.z;
                }
                m_data.*field = temp;
            });
        m_subscribers.push_back(sub);
    }

    void subscribeFloat(const std::string& topic, double DroneData::*field) {
        ros::Subscriber sub = m_nh.subscribe<std_msgs::Float64>(topic, 10,
            [this, field](const std_msgs::Float64::ConstPtr& msg) {
                m_data.*field = msg->data;
            });
        m_subscribers.push_back(sub);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_logger_node");
    ros::NodeHandle nh;

    int num_drones = 1;
    nh.param("/num_drones", num_drones, 1);

    std::vector<std::shared_ptr<DroneLogger>> loggers;
    for (int i = 1; i <= num_drones; ++i) {
        std::string drone_name = "drone" + std::to_string(i);
        loggers.push_back(std::make_shared<DroneLogger>(drone_name));
    }

    ros::spin();
    return 0;
}