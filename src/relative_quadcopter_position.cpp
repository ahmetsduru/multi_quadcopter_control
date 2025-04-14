#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Vector3.h>
#include <map>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>

class DroneDisturbanceManager {
public:
    DroneDisturbanceManager() : rate_(500.0), topic_check_interval_(1.0), drone_count_logged_(false), n_(2.0) {
        k_matrix_ << 0.04, 0.0, 0.0,
                     0.0, 0.04, 0.0,
                     0.0, 0.0, 0.1;

        for (int i = 1; i <= 10; ++i) {
            std::string drone_name = "drone" + std::to_string(i);
            std::string ns = "/" + drone_name + "/";

            geometry_msgs::Vector3 offset;
            nh_.getParam(ns + "initial_x", offset.x);
            nh_.getParam(ns + "initial_y", offset.y);
            nh_.getParam(ns + "initial_z", offset.z);
            drone_initial_offsets_[drone_name] = offset;
        }

        last_topic_check_ = ros::Time::now();
    }

    void run() {
        while (ros::ok()) {
            checkNewTopics();
            logDroneCount();
            computeAndPublishDisturbances();

            ros::spinOnce();
            rate_.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Time last_topic_check_;
    ros::Duration topic_check_interval_;
    bool drone_count_logged_;
    double n_;
    Eigen::Matrix3d k_matrix_;

    std::map<std::string, geometry_msgs::Vector3> drone_positions_;
    std::map<std::string, ros::Subscriber> subscribers_;
    std::map<std::string, geometry_msgs::Vector3> drone_initial_offsets_;
    std::map<std::string, ros::Publisher> disturbance_publishers_;

    std::vector<std::string> getDronePositionTopics() {
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        std::vector<std::string> drone_topics;
        for (const auto& topic : master_topics) {
            if (topic.name.find("/actual_position") != std::string::npos &&
                topic.datatype == "geometry_msgs/Vector3") {
                drone_topics.push_back(topic.name);
            }
        }
        return drone_topics;
    }

    std::string extractDroneName(const std::string& topic_name) {
        size_t first_slash = topic_name.find("/");
        size_t second_slash = topic_name.find("/", first_slash + 1);
        if (first_slash != std::string::npos && second_slash != std::string::npos) {
            return topic_name.substr(first_slash + 1, second_slash - first_slash - 1);
        }
        return topic_name;
    }

    void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& topic_name) {
        std::string drone_name = extractDroneName(topic_name);
        geometry_msgs::Vector3 offset = drone_initial_offsets_[drone_name];

        geometry_msgs::Vector3 adjusted_position;
        adjusted_position.x = msg->x + offset.x;
        adjusted_position.y = msg->y + offset.y;
        adjusted_position.z = msg->z + offset.z;

        drone_positions_[drone_name] = adjusted_position;
    }

    double computeDistance(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) +
                         std::pow(a.y - b.y, 2) +
                         std::pow(a.z - b.z, 2));
    }

    double computeVectorAngleWorldFrame(const geometry_msgs::Vector3& origin, const geometry_msgs::Vector3& head) {
        double dx = head.x - origin.x;
        double dy = head.y - origin.y;
        double dz = head.z - origin.z;

        double horizontal_length = std::sqrt(dx * dx + dy * dy);
        return std::atan2(horizontal_length, dz);
    }

    geometry_msgs::Vector3 unitVector(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
        geometry_msgs::Vector3 result;
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dz = b.z - a.z;
        double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (norm > 1e-6) {
            result.x = dx / norm;
            result.y = dy / norm;
            result.z = dz / norm;
        } else {
            result.x = result.y = result.z = 0.0;
        }
        return result;
    }

    void checkNewTopics() {
        if ((ros::Time::now() - last_topic_check_) > topic_check_interval_) {
            std::vector<std::string> drone_topics = getDronePositionTopics();
            for (const auto& topic : drone_topics) {
                if (subscribers_.find(topic) == subscribers_.end()) {
                    subscribers_[topic] = nh_.subscribe<geometry_msgs::Vector3>(
                        topic, 10, boost::bind(&DroneDisturbanceManager::positionCallback, this, _1, topic));
                    ROS_INFO("Subscribed to topic: %s", topic.c_str());

                    std::string drone_name = extractDroneName(topic);
                    std::string disturbance_topic = "/" + drone_name + "/disturbance_force";
                    disturbance_publishers_[drone_name] = nh_.advertise<geometry_msgs::Vector3>(disturbance_topic, 10);
                }
            }
            last_topic_check_ = ros::Time::now();
        }
    }

    void logDroneCount() {
        if (!drone_count_logged_ && !subscribers_.empty()) {
            ROS_INFO("Number of active drones: %zu", subscribers_.size());
            drone_count_logged_ = true;
        }
    }

    void computeAndPublishDisturbances() {
        std::map<std::string, Eigen::Vector3d> disturbance_accumulator;
        for (const auto& pair : drone_positions_) {
            disturbance_accumulator[pair.first] = Eigen::Vector3d::Zero();
        }

        for (const auto& drone1 : drone_positions_) {
            for (const auto& drone2 : drone_positions_) {
                if (drone1.first >= drone2.first) continue;

                std::string top_drone = drone1.second.z > drone2.second.z ? drone1.first : drone2.first;
                std::string bottom_drone = drone1.second.z > drone2.second.z ? drone2.first : drone1.first;

                double angle_rad = computeVectorAngleWorldFrame(drone_positions_[bottom_drone], drone_positions_[top_drone]);
                double angle_deg = angle_rad * 180.0 / M_PI;

                if (angle_deg <= 50.0) {
                    double distance = computeDistance(drone_positions_[bottom_drone], drone_positions_[top_drone]);
                    geometry_msgs::Vector3 r_ij = unitVector(drone_positions_[bottom_drone], drone_positions_[top_drone]);
                    Eigen::Vector3d r_ij_vec(r_ij.x, r_ij.y, r_ij.z);

                    Eigen::Vector3d force_vec = (-std::cos(angle_rad) / std::pow(distance, n_)) * k_matrix_ * r_ij_vec;
                    disturbance_accumulator[bottom_drone] += force_vec;
                }
            }
        }

        for (const auto& pair : disturbance_accumulator) {
            geometry_msgs::Vector3 force;
            force.x = pair.second(0);
            force.y = pair.second(1);
            force.z = pair.second(2);
            if (disturbance_publishers_.find(pair.first) != disturbance_publishers_.end()) {
                disturbance_publishers_[pair.first].publish(force);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "relative_disturbance_generator");
    DroneDisturbanceManager manager;
    manager.run();
    return 0;
}
