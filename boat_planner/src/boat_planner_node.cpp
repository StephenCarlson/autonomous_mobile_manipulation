#include <ros/ros.h>
// #include "ros/ros.h"

#include <boat_planner/MGI_BoatPlanner.h>

// #include <moveit/move_group/node_name.h>
// 
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// 
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// 
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>
// 
// #include <moveit_visual_tools/moveit_visual_tools.h>

// #include <sensor_msgs/Joy.h>
// #include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "boat_planner_node");
    
    // ros::NodeHandle nh; // Public Node Handle
    // ros::NodeHandle nh_;
    // ros::NodeHandle private_nh_;
    // std::string robot_namespace;
    // private_nh_.getParam("robot_namespace", robot_namespace);
    // std::string arm_namespace;
    // private_nh_.getParam("arm_namespace", arm_namespace);
    // tf_listener_ = std::make_unique<tf::TransformListener>();
    // // Setup Visualization
    // visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(robot_namespace + "/bvr_base_link");
    // visual_tools_->deleteAllMarkers();


    // ros::AsyncSpinner spinner(2); // TODO: Examples have 1, but move_group fails with: "Didn't received robot state (joint angles) with recent timestamp within 1 seconds." (see: https://github.com/ros-planning/moveit/issues/868)
    // spinner.start();

    
    // ros::NodeHandle pnh("~");

    // move_group_interface::MGI_AprilTag move_group_interface_AprilTag(nh, pnh, "map");
  
    // ros::waitForShutdown();


    ROS_INFO("boat_planner is properly compiled if this message is visible.");

    // http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

    // typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    // MoveBaseClient ac("move_base", true);
    // while(!ac.waitForServer(ros::Duration(5.0))) {
    //     ROS_INFO("Waiting for the move_base action server to come up");
    // }
    // move_base_msgs::MoveBaseGoal goal;
    // goal.target_pose.header.frame_id = "bvr_SIM/move_base";
    // goal.target_pose.header.stamp = ros::Time::now();
    // goal.target_pose.pose.position.x = 1.0;
    // goal.target_pose.pose.orientation.w = 1.0;
    // ROS_INFO("Sending goal");
    // ac.sendGoal(goal);
    // ac.waitForResult();
    // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_INFO("Test movement succeeded");
    // } else {
    //     ROS_INFO("Test movement failed");
    // }

    // ros::spin();


    // Hopefully this is as good as any place to put the instructions:
    // Terminal 1: cd /autonomous_mobile_manipulation_ws, source devel/setup.bash .
    // Then: roslaunch robowork_gazebo bvr_SIM_playpen.launch
    // Terminal 2: Same steps of cd and source, then:
    // ROS_NAMESPACE="bvr_SIM" roslaunch robowork_moveit_config robowork_moveit_planning_execution.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM
    // Terminal 3: Same steps of cd and source, then:
    // ROS_NAMESPACE="bvr_SIM" roslaunch boat_planner main.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM
    // The arm should move to the point infront of the robot.





    ros::AsyncSpinner spinner(2); // TODO: Examples have 1, but move_group fails with: "Didn't received robot state (joint angles) with recent timestamp within 1 seconds." (see: https://github.com/ros-planning/moveit/issues/868)
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    move_group_interface::MGI_BoatPlanner move_group_interface_BoatPlanner(nh, pnh, "map");

    // move_group_interface::MGI_BoatPlanner hardcoded_3d_cb();
  
    ros::waitForShutdown();
    return 0;
}