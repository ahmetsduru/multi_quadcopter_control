#include <ros/ros.h>
#include "../include/multi_quadcopter_control/low_level_cont.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_low_level_controller_node");

    LowLevelController low_controller;
    low_controller.spin();

    return 0;
}
