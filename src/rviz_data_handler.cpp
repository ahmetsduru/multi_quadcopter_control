#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class RvizDataHandler {
public:
    RvizDataHandler(ros::NodeHandle& nh) : nh_(nh) {
        // Namespace
        drone_ns_ = ros::this_node::getNamespace();
        if (drone_ns_ == "/" || drone_ns_.empty()) drone_ns_ = "";
        else if (drone_ns_.front() == '/') drone_ns_.erase(0, 1);

        // Publishers
        path_pub_ = nh_.advertise<nav_msgs::Path>("actual_position_path", 10);
        ref_path_pub_ = nh_.advertise<nav_msgs::Path>("reference_position_path", 10);

        // Initial offset
        nh_.param("initial_x", initial_position_.x, 0.0);
        nh_.param("initial_y", initial_position_.y, 0.0);
        nh_.param("initial_z", initial_position_.z, 0.0);

        // Subscribers
        pos_sub_ = nh_.subscribe<geometry_msgs::Vector3>("actual_position", 10, 
            [this](const geometry_msgs::Vector3::ConstPtr& msg) {
                this->actualPositionCallback(msg);
            });

        euler_sub_ = nh_.subscribe("actual_euler_angles", 10, &RvizDataHandler::actualEulerCallback, this);
        ref_pos_sub_ = nh_.subscribe("reference_position", 10, &RvizDataHandler::referencePositionCallback, this);
        dummy_sub_ = nh_.subscribe("waypoints", 1, &RvizDataHandler::dummyCallback, this);

        tf_broadcaster_ptr_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    }

    void spin() {
        ros::Rate rate(500);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_msg_;
    ros::Publisher ref_path_pub_;
    nav_msgs::Path ref_path_msg_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;
    ros::Subscriber pos_sub_, euler_sub_, ref_pos_sub_, dummy_sub_;

    geometry_msgs::Vector3 latest_euler_angles_;
    geometry_msgs::Vector3 initial_position_;
    std::string drone_ns_;

    void actualEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        latest_euler_angles_ = *msg;
    }

    void actualPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        ros::Time current_time = ros::Time::now();

        tf2::Quaternion quat;
        quat.setRPY(latest_euler_angles_.x, latest_euler_angles_.y, latest_euler_angles_.z);
        quat.normalize();

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = current_time;
        pose.header.frame_id = "world";
        pose.pose.position.x = msg->x + initial_position_.x;
        pose.pose.position.y = msg->y + initial_position_.y;
        pose.pose.position.z = msg->z + initial_position_.z;
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        if (path_msg_.header.frame_id.empty()) {
            path_msg_.header.frame_id = "world";
        }
        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = current_time;
        path_pub_.publish(path_msg_);

        if (tf_broadcaster_ptr_) {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = current_time;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = drone_ns_ + "/base_link";
            transformStamped.transform.translation.x = pose.pose.position.x;
            transformStamped.transform.translation.y = pose.pose.position.y;
            transformStamped.transform.translation.z = pose.pose.position.z;
            transformStamped.transform.rotation = pose.pose.orientation;
            tf_broadcaster_ptr_->sendTransform(transformStamped);
        }
    }

    void referencePositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        ros::Time current_time = ros::Time::now();

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = current_time;
        pose.header.frame_id = "world";
        pose.pose.position.x = msg->x + initial_position_.x;
        pose.pose.position.y = msg->y + initial_position_.y;
        pose.pose.position.z = msg->z + initial_position_.z;

        if (ref_path_msg_.header.frame_id.empty()) {
            ref_path_msg_.header.frame_id = "world";
        }
        ref_path_msg_.poses.push_back(pose);
        ref_path_msg_.header.stamp = current_time;
        ref_path_pub_.publish(ref_path_msg_);
    }

    void dummyCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
        // No-op
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_data_handler");
    ros::NodeHandle nh;
    RvizDataHandler handler(nh);
    handler.spin();
    return 0;
}