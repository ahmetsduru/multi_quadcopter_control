#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Vector3.h>
#include <map>
#include <cmath>
#include <vector>
#include <string>

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

// Stores the latest position of each drone
std::map<std::string, geometry_msgs::Vector3> drone_positions;
// Keeps track of active subscribers
std::map<std::string, ros::Subscriber> subscribers;
// Stores initial position offsets per drone
std::map<std::string, geometry_msgs::Vector3> drone_initial_offsets;

// Extracts drone name from the topic path
std::string extractDroneName(const std::string& topic_name) {
    size_t first_slash = topic_name.find("/");
    size_t second_slash = topic_name.find("/", first_slash + 1);
    if (first_slash != std::string::npos && second_slash != std::string::npos) {
        return topic_name.substr(first_slash + 1, second_slash - first_slash - 1);
    }
    return topic_name;
}

// Callback function to store position data for each drone
void positionCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& topic_name) {
    std::string drone_name = extractDroneName(topic_name);
    geometry_msgs::Vector3 offset = drone_initial_offsets[drone_name];

    geometry_msgs::Vector3 adjusted_position;
    adjusted_position.x = msg->x + offset.x;
    adjusted_position.y = msg->y + offset.y;
    adjusted_position.z = msg->z + offset.z;

    drone_positions[drone_name] = adjusted_position;
}

// Compute distance between two vectors
double computeDistance(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) +
                     std::pow(a.y - b.y, 2) +
                     std::pow(a.z - b.z, 2));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_drone_position_listener");
    ros::NodeHandle nh;

    ros::Rate rate(10.0);
    ros::Time last_topic_check = ros::Time::now();
    ros::Duration topic_check_interval(1.0);

    bool drone_count_logged = false;

    // Retrieve initial offset parameters for each drone
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
        // Check periodically for new drone topics and subscribe to them
        if ((ros::Time::now() - last_topic_check) > topic_check_interval) {
            std::vector<std::string> drone_topics = getDronePositionTopics();
            for (const auto& topic : drone_topics) {
                if (subscribers.find(topic) == subscribers.end()) {
                    subscribers[topic] = nh.subscribe<geometry_msgs::Vector3>(
                        topic, 10, boost::bind(positionCallback, _1, topic));
                    ROS_INFO("Subscribed to topic: %s", topic.c_str());
                }
            }
            last_topic_check = ros::Time::now();
        }

        if (!drone_count_logged && !subscribers.empty()) {
            ROS_INFO("Number of active drones: %zu", subscribers.size());
            drone_count_logged = true;
        }

        // Check distances between drone pairs and identify which drone is above or below
        for (const auto& drone1 : drone_positions) {
            for (const auto& drone2 : drone_positions) {
                if (drone1.first >= drone2.first) continue;  // Avoid duplicate pairs

                bool drone1_above_drone2 = drone1.second.z > drone2.second.z;

                double distance = computeDistance(drone1.second, drone2.second);
                if (distance < 0.5) {
                    ROS_INFO("CLOSE PAIR -> Top: %s | Bottom: %s | Distance: %.2f m",
                             drone1_above_drone2 ? drone1.first.c_str() : drone2.first.c_str(),
                             drone1_above_drone2 ? drone2.first.c_str() : drone1.first.c_str(),
                             distance);
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}