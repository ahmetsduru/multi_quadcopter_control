#include <ros/ros.h>
#include "../include/multi_quadcopter_control/waypoint_server.h"  // Include the header file for WaypointServer

int main(int argc, char** argv) {

    ros::init(argc, argv, "waypoint_server");

    ros::NodeHandle nh;

    WaypointServer traj_server(nh);

    ros::spin();

    return 0;
}
