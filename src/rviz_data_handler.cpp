#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher path_pub;         // Drone için Path Publisher
nav_msgs::Path path_msg;         // Drone için Path mesajı
tf2_ros::TransformBroadcaster* tf_broadcaster_ptr = nullptr; // TF yayıncısı için pointer

void referencePositionCallback(const geometry_msgs::Vector3::ConstPtr& msg, const std::string& drone_ns) {
    ros::Time current_time = ros::Time::now();  // Sürekli güncellenen zaman

    // Drone için yeni bir PoseStamped nesnesi oluştur
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = current_time;
    pose.header.frame_id = "world";  // RViz'de kullanılan çerçeve adı
    pose.pose.position.x = msg->x;
    pose.pose.position.y = msg->y;
    pose.pose.position.z = msg->z;

    // Path mesajına yeni noktayı ekle
    path_msg.poses.push_back(pose);
    path_msg.header.stamp = current_time;
    path_pub.publish(path_msg);

    // TF Yayını
    if (tf_broadcaster_ptr) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = drone_ns + "/base_link";
        transformStamped.transform.translation.x = msg->x;
        transformStamped.transform.translation.y = msg->y;
        transformStamped.transform.translation.z = msg->z;
        transformStamped.transform.rotation.w = 1.0;
        tf_broadcaster_ptr->sendTransform(transformStamped);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rviz_data_handler");
    ros::NodeHandle nh;

    // Path için publisher başlat
    std::string drone_ns = ros::this_node::getNamespace();
    if (drone_ns == "/" || drone_ns.empty()) drone_ns = "";
    else if (drone_ns.front() == '/') drone_ns.erase(0, 1);
    
    path_pub = nh.advertise<nav_msgs::Path>("reference_position_path", 10);

    // Vector3 türündeki veriyi dinleyen subscriber başlat
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Vector3>("reference_position", 10, 
        [&drone_ns](const geometry_msgs::Vector3::ConstPtr& msg) {
            referencePositionCallback(msg, drone_ns);
        });

    // TF2 Yayıncısını oluştur
    tf2_ros::TransformBroadcaster tf_broadcaster;
    tf_broadcaster_ptr = &tf_broadcaster;

    ros::spin();
    return 0;
}
