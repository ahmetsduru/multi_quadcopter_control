#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ros::Publisher path_pub;         // Path Publisher for the drone
nav_msgs::Path path_msg;         // Path message for the drone
ros::Publisher ref_path_pub;     // Reference Path Publisher
nav_msgs::Path ref_path_msg;     // Reference Path message
tf2_ros::TransformBroadcaster* tf_broadcaster_ptr = nullptr; // Pointer for TF broadcaster
ros::Subscriber dummy_sub;       // Dummy Subscriber

geometry_msgs::Vector3 latest_euler_angles;  // Latest updated Euler angles
geometry_msgs::Vector3 initial_position;     // Initial Position Offset

void actualEulerCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    latest_euler_angles = *msg;  // Update Euler angles
}

void actualPositionCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& drone_ns) {
    ros::Time current_time = ros::Time::now();  // Continuously updated time

    // 🛠 **Compute Quaternion from Euler Angles**
    tf2::Quaternion quat;
    quat.setRPY(latest_euler_angles.x, latest_euler_angles.y, latest_euler_angles.z);
    quat.normalize(); // Normalize to prevent errors

    //  **Create and publish PoseStamped**
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = current_time;
    pose.header.frame_id = "world";  // Frame name used in RViz
    pose.pose.position.x = msg->x + initial_position.x;
    pose.pose.position.y = msg->y + initial_position.y;
    pose.pose.position.z = msg->z + initial_position.z;

    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    //  **Check the frame_id of the path message**
    if (path_msg.header.frame_id.empty()) {
        path_msg.header.frame_id = "world";
    }

    //  **Append the new point to the Path message and publish it**
    path_msg.poses.push_back(pose);
    path_msg.header.stamp = current_time;
    path_pub.publish(path_msg);

    //  **TF Broadcasting**
    if (tf_broadcaster_ptr) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = drone_ns + "/base_link";

        transformStamped.transform.translation.x = msg->x + initial_position.x;
        transformStamped.transform.translation.y = msg->y + initial_position.y;
        transformStamped.transform.translation.z = msg->z + initial_position.z;

        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster_ptr->sendTransform(transformStamped);
    }
}

void referencePositionCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    ros::Time current_time = ros::Time::now();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = current_time;
    pose.header.frame_id = "world";
    pose.pose.position.x = msg->x + initial_position.x;
    pose.pose.position.y = msg->y + initial_position.y;
    pose.pose.position.z = msg->z + initial_position.z;

    // **Check the frame_id of the Reference Path message**
    if (ref_path_msg.header.frame_id.empty()) {
        ref_path_msg.header.frame_id = "world";
    }

    //  **Append the new point to the Reference Path message and publish it**
    ref_path_msg.poses.push_back(pose);
    ref_path_msg.header.stamp = current_time;
    ref_path_pub.publish(ref_path_msg);
}

//  **Dummy Callback (Does nothing but makes the subscriber active)**
void dummyCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    // Do nothing, just trick ROS into thinking there is an active subscriber
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_data_handler");
    ros::NodeHandle nh;

    // **Initialize the path publisher**
    std::string drone_ns = ros::this_node::getNamespace();
    if (drone_ns == "/" || drone_ns.empty()) drone_ns = "";
    else if (drone_ns.front() == '/') drone_ns.erase(0, 1);
    
    path_pub = nh.advertise<nav_msgs::Path>("actual_position_path", 10);
    ref_path_pub = nh.advertise<nav_msgs::Path>("reference_position_path", 10);

    // **Read Initial Position Offset from Parameter Server**
    nh.param("initial_x", initial_position.x, 1.0);
    nh.param("initial_y", initial_position.y, 0.0);
    nh.param("initial_z", initial_position.z, 0.0);

    // **Start subscribers for actual_position, actual_euler_angles, and reference_position**
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::Vector3>("actual_position", 10, 
        [&drone_ns](const geometry_msgs::Vector3::ConstPtr& msg) {
            actualPositionCallback(msg, drone_ns);
        });

    ros::Subscriber euler_sub = nh.subscribe<geometry_msgs::Vector3>("actual_euler_angles", 10, 
        actualEulerCallback);
    
    ros::Subscriber ref_pos_sub = nh.subscribe<geometry_msgs::Vector3>("reference_position", 10, 
        referencePositionCallback);

    // **Create the TF2 broadcaster**
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf_broadcaster_ptr = &tf_broadcaster;

    // **Add Dummy Subscriber**
    dummy_sub = nh.subscribe("waypoints", 1, dummyCallback);

    ros::Rate rate(500);  // **500 Hz**
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
