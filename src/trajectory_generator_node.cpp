#include "../include/multi_quadcopter_control/trajectory_generator.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
   
    ros::init(argc, argv, "trajectory_generator_node");
    
    ros::NodeHandle nh;

    TrajectoryGenerator traj_gen(nh);

    traj_gen.generateTrajectory();

    ros::spin();

    return 0;
}
