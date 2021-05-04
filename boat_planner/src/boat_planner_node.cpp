#include <ros/ros.h>
// #include "ros/ros.h"
#include <std_msgs/Float64.h>

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


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void move_base( move_base_msgs::MoveBaseGoal goal)
{
    // I notice that this is directly taken from http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
    MoveBaseClient ac("bvr_SIM/move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    // move_base_msgs::MoveBaseGoal goal;
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("move base goal success");
    else
    ROS_INFO("The base failed to move for some reason");
}

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



    // ros::Subscriber base_sub;
    // ros::Subscriber arm_sub;

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

    // move_group_interface::MGI_BoatPlanner move_group_interface_BoatPlanner(nh, pnh, "map");

    // std::vector<std::map<std::string,std::string>> points_list;
    std::vector<std::vector<double>> waypoints_list;
    XmlRpc::XmlRpcValue xml_points_list;
    nh.getParam("waypoint_list", xml_points_list);
    // ROS_INFO("XML Type: %d", xml_points_list.getType());
    if(xml_points_list.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
        ROS_ERROR("param 'waypoint_list' is not a list");
    } else {
        for(int i=0; i<xml_points_list.size(); i++) {
            // ROS_INFO("Line %d: XML Type: %d", i, xml_points_list[i].getType());
            if( xml_points_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray) {
            // if( xml_points_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR("waypoint_list[%d] is not a struct", i);
            } else {
                // ROS_INFO("Line %d: XML Type: %d", i, xml_points_list[i]["base_pos_x"].getType());
                // ROS_INFO("Line %d: XML Type: %d", i, xml_points_list[i].hasMember("base_pos_x"));

                std::vector<double> waypoint;

                // waypoint.push_back(xml_points_list[i]["base_pos_x"]);
                // waypoint.push_back(xml_points_list[i]["base_pos_y"]);
                // waypoint.push_back(xml_points_list[i]["eff_pos_x"]);
                // waypoint.push_back(xml_points_list[i]["eff_pos_y"]);
                // waypoint.push_back(xml_points_list[i]["eff_pos_z"]);
                // waypoint.push_back(xml_points_list[i]["eff_orint_x"]);
                // waypoint.push_back(xml_points_list[i]["eff_orint_y"]);
                // waypoint.push_back(xml_points_list[i]["eff_orint_z"]);
                // waypoint.push_back(xml_points_list[i]["eff_orint_w"]);

                waypoint.push_back(xml_points_list[i][0]);
                waypoint.push_back(xml_points_list[i][1]);
                waypoint.push_back(xml_points_list[i][2]);
                // waypoint.push_back(xml_points_list[i][3]);
                // waypoint.push_back(xml_points_list[i][4]);
                // waypoint.push_back(xml_points_list[i][5]);
                // waypoint.push_back(xml_points_list[i][6]);
                // waypoint.push_back(xml_points_list[i][7]);
                // waypoint.push_back(xml_points_list[i][8]);

                waypoints_list.push_back(waypoint);
            }
        }
    }

    // for (const auto & line : waypoints_list) {
    //     for(const auto & value : line) {
    //         std::cout << " - " << value;
    //     }
    //     std::cout << std::endl;
    //     // ROS_INFO("YAML Parsed? %s", line);
    // }



    // ros::Publisher testPublisher = nh.advertise<std_msgs::Float64>("/waypoint_goal", 10);
    // ros::Rate loop_rate(10);
    // int start_time, elapsed;
    // while (not start_time) {
    //     start_time = ros::Time::now().toSec();
    // }
    // while (ros::ok()) {
    //     elapsed = ros::Time::now().toSec() - start_time;
    //     std_msgs::Float64 testMessage;
    //     testMessage.data = elapsed * 1.0f;
    //     testPublisher.publish(testMessage);
    //     loop_rate.sleep();
    // }
    // Giving up on trying to make a service or publisher, just pull everything from main() here
    // move_group_interface::MGI_BoatPlanner hardcoded_3d_cb();

    // geometry_msgs::PoseStamped curr_pose = move_group_->getCurrentPose();
    move_base_msgs::MoveBaseGoal goal_base;
    // geometry_msgs::PoseStamped goal_pose = curr_pose;

    // std::vector<double> prevMoveBase;

    for (const auto & line : waypoints_list) {

        goal_base.target_pose.pose.position.x = line[0];
        goal_base.target_pose.pose.position.y = line[1];
        // tf::Quaternion quat_b =  armToBaseOrientation(base_pose);
        // goal_base.target_pose.pose.orientation.x = quat_b.x();
        // goal_base.target_pose.pose.orientation.y = quat_b.y();
        // goal_base.target_pose.pose.orientation.z = quat_b.z();
        goal_base.target_pose.pose.orientation.w = 1.0f;
        goal_base.target_pose.header.frame_id = "map";
        ROS_INFO_STREAM("Moving Base to goal: "<< goal_base.target_pose.pose);
        // visual_tools.publishArrow(goal_base.target_pose.pose,rviz_visual_tools::colors::RED,rviz_visual_tools::scales::LARGE);
        // visual_tools.trigger();
        move_base(goal_base);


        // auto arm_pose = ArmBasePairs.arm_poses.poses[ind];
        // geometry_msgs::Pose
        // tf::Pose(tf::Quaternion(0,0,0,1), tf::Point(1,0,0.5))
        // geometry_msgs::Pose arm_pose = geometry_msgs::Pose
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // move_group_->setStartState(*move_group_->getCurrentState());
        // goal_pose.pose = arm_pose;
        // goal_pose.header.stamp = ros::Time::now();
        // goal_pose.header.frame_id = "map";
        // move_group_->setGoalOrientationTolerance(0.23);
        // move_group_->setGoalPositionTolerance(0.05);
        // move_group_->setPoseTarget(goal_pose);
        // // visual_tools.publishArrow(goal_pose.pose,rviz_visual_tools::colors::YELLOW,rviz_visual_tools::scales::XLARGE);
        // // visual_tools.trigger();
        // // visual_tools.deleteAllMarkers();
        // success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if(success)
        // {
        //     ROS_INFO_STREAM("plan success for arm pose" << arm_pose);
        //     // visual_tools.prompt("Found solution Press 'next'to execute trajectory");
        //     moveit::planning_interface::MoveItErrorCode exec_success = move_group_->execute(my_plan);
        //     ROS_INFO_STREAM("execution success message #"<< exec_success.val);
        // }
        // else
        // {
        //     move_group_->setGoalOrientationTolerance(0.449066);
        //     move_group_->setGoalPositionTolerance(0.2);
        //     success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        //     if(success)
        //     {
        //         ROS_INFO_STREAM("plan success for arm pose" << arm_pose);
        //         visual_tools.prompt("Found solution Press 'next'to execute trajectory");
        //         moveit::planning_interface::MoveItErrorCode exec_success = move_group_->execute(my_plan);
        //         ROS_INFO_STREAM("execution success message #"<< exec_success.val);
        //     }
        // }



        // if(line != prevMoveBase) {
        // prevMoveBase = line;
        // }


    }






  
    ros::waitForShutdown();
    return 0;
}