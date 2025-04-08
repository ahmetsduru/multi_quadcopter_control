#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Vector3.h>
#include <map>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense> // For vector angle calculation

// Retrieves all active drone position topics
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

std::map<std::string, geometry_msgs::Vector3> drone_positions;
std::map<std::string, ros::Subscriber> subscribers;
std::map<std::string, geometry_msgs::Vector3> drone_initial_offsets;
std::map<std::string, ros::Publisher> disturbance_publishers;

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
    geometry_msgs::Vector3 offset = drone_initial_offsets[drone_name];

    geometry_msgs::Vector3 adjusted_position;
    adjusted_position.x = msg->x + offset.x;
    adjusted_position.y = msg->y + offset.y;
    adjusted_position.z = msg->z + offset.z;

    drone_positions[drone_name] = adjusted_position;
}

double computeDistance(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) +
                     std::pow(a.y - b.y, 2) +
                     std::pow(a.z - b.z, 2));
}

double computeVectorAngleWorldFrame(const geometry_msgs::Vector3& origin, const geometry_msgs::Vector3& head) {
    // Calculate angle between Z-axis and XY projection of the vector
    double dx = head.x - origin.x;
    double dy = head.y - origin.y;
    double dz = head.z - origin.z;

    double horizontal_length = std::sqrt(dx * dx + dy * dy);

    // Angle between Z axis and the vector
    double angle = std::atan2(horizontal_length, dz); // angle from Z-axis down to XY plane

    return angle;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_drone_position_listener");
    ros::NodeHandle nh;

    ros::Rate rate(500.0);
    ros::Time last_topic_check = ros::Time::now();
    ros::Duration topic_check_interval(1.0);

    bool drone_count_logged = false;

    for (int i = 1; i <= 10; ++i) {
        std::string drone_name = "drone" + std::to_string(i);
        std::string ns = "/" + drone_name + "/";

        geometry_msgs::Vector3 offset;
        nh.getParam(ns + "initial_x", offset.x);
        nh.getParam(ns + "initial_y", offset.y);
        nh.getParam(ns + "initial_z", offset.z);
        drone_initial_offsets[drone_name] = offset;
    }

    while (ros::ok()) {
        if ((ros::Time::now() - last_topic_check) > topic_check_interval) {
            std::vector<std::string> drone_topics = getDronePositionTopics();
            for (const auto& topic : drone_topics) {
                if (subscribers.find(topic) == subscribers.end()) {
                    subscribers[topic] = nh.subscribe<geometry_msgs::Vector3>(
                        topic, 10, boost::bind(positionCallback, _1, topic));
                    ROS_INFO("Subscribed to topic: %s", topic.c_str());

                    std::string drone_name = extractDroneName(topic);
                    std::string disturbance_topic = "/" + drone_name + "/disturbance_force";
                    disturbance_publishers[drone_name] = nh.advertise<geometry_msgs::Vector3>(disturbance_topic, 10);
                }
            }
            last_topic_check = ros::Time::now();
        }

        if (!drone_count_logged && !subscribers.empty()) {
            ROS_INFO("Number of active drones: %zu", subscribers.size());
            drone_count_logged = true;
        }

        std::map<std::string, geometry_msgs::Vector3> disturbances;
        for (const auto& pair : drone_positions) {
            geometry_msgs::Vector3 zero_force;
            zero_force.x = 0.0;
            zero_force.y = 0.0;
            zero_force.z = 0.0;
            disturbances[pair.first] = zero_force;
        }

        for (const auto& drone1 : drone_positions) {
            for (const auto& drone2 : drone_positions) {
                if (drone1.first >= drone2.first) continue;

                double distance = computeDistance(drone1.second, drone2.second);
                if (distance <= 0.5) {
                    std::string top_drone = drone1.second.z > drone2.second.z ? drone1.first : drone2.first;
                    std::string bottom_drone = drone1.second.z > drone2.second.z ? drone2.first : drone1.first;

                    double angle_rad = computeVectorAngleWorldFrame(drone_positions[bottom_drone], drone_positions[top_drone]);
                    double angle_deg = angle_rad * 180.0 / M_PI;
                    ROS_INFO("Vector angle (Bottom to Top) in world frame: %.2f degrees | Bottom Drone: %s | Top Drone: %s | Distance: %.2f m",
         angle_deg, bottom_drone.c_str(), top_drone.c_str(), distance);

                    if (angle_deg < 40.0) {
                        //ROS_INFO("Applying disturbance force: angle=%.2f deg, distance=%.2f m", angle_deg, distance);
                        geometry_msgs::Vector3 force;
                        force.x = 0.0;
                        force.y = 0.0;
                        force.z = -2.0;
                        disturbances[bottom_drone] = force;
                    }
                }
            }
        }

        for (const auto& pair : disturbances) {
            if (disturbance_publishers.find(pair.first) != disturbance_publishers.end()) {
                disturbance_publishers[pair.first].publish(pair.second);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
