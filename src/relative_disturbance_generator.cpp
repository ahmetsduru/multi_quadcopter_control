#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/Vector3.h>
#include <map>
#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <random>

class DroneDisturbanceManager {
public:
    DroneDisturbanceManager() : rate_(500.0), topic_check_interval_(1.0), drone_count_logged_(false) {
        
        // --- 1. Model Parametreleri (Dinamik Hesaplama) ---
        double m_drone;
        if (nh_.getParam("/state_derivative_solver_node/mass", m_drone)) {
            ROS_INFO("Disturbance Manager: Mass loaded from config: %.4f kg", m_drone);
        } else {
            m_drone = 0.382; 
            ROS_WARN("Disturbance Manager: Mass parameter not found, using default: %.4f kg", m_drone);
        }

        double g = 9.81;
        double c_coup = 0.045; 
        double r_lat = 0.005;  

        // Fiziksel Hesaplama
        double thrust_total = m_drone * g;
        double thrust_per_rotor = thrust_total / 4.0;

        double k_z = thrust_per_rotor * c_coup; 
        double k_x = k_z * r_lat;           
        double k_y = k_z * r_lat;

        k_matrix_ << k_x, 0.0, 0.0,
                     0.0, k_y, 0.0,
                     0.0, 0.0, k_z;

        ROS_INFO("Calculated K Matrix -> K_z: %.6f, K_x/y: %.6f", k_z, k_x);

        // --- SABİTLER ---
        z0_ = 0.1;              
        sigma_ = 0.15;           
        turb_intensity_ = 0.2;  
        
        // Bu değerden uzaktaki dronelar birbirini etkilemez kabul edilir (Communication Limit).
        max_effect_radius_ = 1.0; 

        // Random Number Generator
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::uniform_real_distribution<double>(-1.0, 1.0); 

        // Offsetleri al
        for (int i = 1; i <= 20; ++i) { 
            std::string drone_name = "drone" + std::to_string(i);
            std::string ns = "/" + drone_name + "/";

            geometry_msgs::Vector3 offset;
            if (nh_.getParam(ns + "initial_x", offset.x)) {
                nh_.getParam(ns + "initial_y", offset.y);
                nh_.getParam(ns + "initial_z", offset.z);
            } else {
                offset.x = 0; offset.y = 0; offset.z = 0;
            }
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
    
    Eigen::Matrix3d k_matrix_;
    double z0_;
    double sigma_;
    double turb_intensity_;
    
    // [EKLEME 2] Member variable tanımı
    double max_effect_radius_;

    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_;

    std::map<std::string, geometry_msgs::Vector3> drone_positions_;
    std::map<std::string, ros::Subscriber> subscribers_;
    std::map<std::string, geometry_msgs::Vector3> drone_initial_offsets_;
    
    std::map<std::string, ros::Publisher> pub_total_;  
    std::map<std::string, ros::Publisher> pub_static_; 

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
        
        geometry_msgs::Vector3 offset;
        if (drone_initial_offsets_.find(drone_name) != drone_initial_offsets_.end()) {
             offset = drone_initial_offsets_[drone_name];
        } else {
             offset.x = 0; offset.y = 0; offset.z = 0;
        }

        geometry_msgs::Vector3 adjusted_position;
        adjusted_position.x = msg->x + offset.x;
        adjusted_position.y = msg->y + offset.y;
        adjusted_position.z = msg->z + offset.z;

        drone_positions_[drone_name] = adjusted_position;
    }

    void checkNewTopics() {
        if ((ros::Time::now() - last_topic_check_) > topic_check_interval_) {
            std::vector<std::string> drone_topics = getDronePositionTopics();
            for (const auto& topic : drone_topics) {
                if (subscribers_.find(topic) == subscribers_.end()) {
                    subscribers_[topic] = nh_.subscribe<geometry_msgs::Vector3>(
                        topic, 10, boost::bind(&DroneDisturbanceManager::positionCallback, this, _1, topic));
                    
                    std::string drone_name = extractDroneName(topic);

                    std::string topic_total = "/" + drone_name + "/disturbance_force";
                    pub_total_[drone_name] = nh_.advertise<geometry_msgs::Vector3>(topic_total, 10);

                    std::string topic_static = "/" + drone_name + "/disturbance_static";
                    pub_static_[drone_name] = nh_.advertise<geometry_msgs::Vector3>(topic_static, 10);
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
        std::map<std::string, Eigen::Vector3d> acc_total;
        std::map<std::string, Eigen::Vector3d> acc_static;

        for (const auto& pair : drone_positions_) {
            acc_total[pair.first] = Eigen::Vector3d::Zero();
            acc_static[pair.first] = Eigen::Vector3d::Zero();
        }

        for (const auto& drone1 : drone_positions_) {
            for (const auto& drone2 : drone_positions_) {
                if (drone1.first == drone2.first) continue;

                std::string top_name, bottom_name;
                Eigen::Vector3d p_top, p_bottom;

                if (drone1.second.z > drone2.second.z) {
                    top_name = drone1.first;
                    bottom_name = drone2.first;
                    p_top = Eigen::Vector3d(drone1.second.x, drone1.second.y, drone1.second.z);
                    p_bottom = Eigen::Vector3d(drone2.second.x, drone2.second.y, drone2.second.z);
                } else {
                    continue; 
                }

                Eigen::Vector3d p_bt = p_top - p_bottom;
                double dist = p_bt.norm();

                // dist > 0.05 (Çakışma önleme)
                // dist < max_effect_radius_ (Ölçeklenebilirlik / Sınırlı İletişim)
                if (dist > 0.05 && dist < max_effect_radius_) {
                    
                    Eigen::Vector3d n_vec = p_bt.normalized();
                    double d_z = std::abs(p_bt.z());
                    double d_xy = std::sqrt(std::pow(p_bt.x(), 2) + std::pow(p_bt.y(), 2));

                    // --- Gaussian Wake (Statik) ---
                    double decay_factor = (1.0 / std::pow(d_z + z0_, 2)) * std::exp(-std::pow(d_xy, 2) / (2.0 * std::pow(sigma_, 2)));
                    Eigen::Vector3d f_static = -k_matrix_ * n_vec * decay_factor;

                    // --- Türbülans ---
                    Eigen::Vector3d random_vec(dist_(gen_), dist_(gen_), dist_(gen_));
                    Eigen::Vector3d f_turb = f_static.cwiseAbs().cwiseProduct(random_vec * turb_intensity_);

                    // 1. Toplam Kuvvet Akümülatörü
                    acc_total[bottom_name] += (f_static + f_turb) * 4.0;

                    // 2. Statik Kuvvet Akümülatörü
                    acc_static[bottom_name] += f_static * 4.0;
                }
            }
        }

        // Hesaplanan kuvvetleri yayınla
        for (const auto& pair : acc_total) {
            std::string d_name = pair.first;

            geometry_msgs::Vector3 msg_total;
            msg_total.x = pair.second.x();
            msg_total.y = pair.second.y();
            msg_total.z = pair.second.z();

            if (pub_total_.find(d_name) != pub_total_.end()) {
                pub_total_[d_name].publish(msg_total);
            }

            geometry_msgs::Vector3 msg_static;
            msg_static.x = acc_static[d_name].x();
            msg_static.y = acc_static[d_name].y();
            msg_static.z = acc_static[d_name].z();

            if (pub_static_.find(d_name) != pub_static_.end()) {
                pub_static_[d_name].publish(msg_static);
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