#include <ros/ros.h>
// #include "ros/ros.h"

// #include <boat_planner/boat_planner.h>

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "boat_planner_node");

    // ros::AsyncSpinner spinner(2); // TODO: Examples have 1, but move_group fails with: "Didn't received robot state (joint angles) with recent timestamp within 1 seconds." (see: https://github.com/ros-planning/moveit/issues/868)
    // spinner.start();

    // ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");

    // move_group_interface::MGI_AprilTag move_group_interface_AprilTag(nh, pnh, "map");
  
    // ros::waitForShutdown();




    ROS_INFO("boat_planner is properly compiled if this message is visible.");

    ros::spin();

    return 0;
}