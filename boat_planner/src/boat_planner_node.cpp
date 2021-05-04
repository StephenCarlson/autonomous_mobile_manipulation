#include <ros/ros.h>
// #include <std_msgs/Float64.h>

// I've had to abandon trying to do everything in MGI_BoatPlanner.cpp like a well-formed project.
// #include <boat_planner/MGI_BoatPlanner.h>

#include <moveit/move_group/node_name.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::string PLANNING_GROUP_;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
std::unique_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
std::unique_ptr<tf::TransformListener> tf_listener_;
// std::unique_ptr<MoveBaseClient> ac;

const robot_state::JointModelGroup* joint_model_group_;




void move_base( move_base_msgs::MoveBaseGoal goal)
{
    // I notice that this is directly taken from http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
    // MoveBaseClient ac("bvr_SIM/move_base", true);
    MoveBaseClient ac("move_base", true);
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






    // PLANNING_GROUP_ = "main_arm_SIM";
    // std::string robot_namespace = "bvr_SIM";
    // std::string arm_namespace = "main_arm_SIM";
    // // node_handle.getParam("robot_namespace", robot_namespace);
    // // node_handle.getParam("arm_namespace", arm_namespace);
    // std::string bvr_base_interia_frame = robot_namespace+"/bvr_base_inertia";
    // std::string gripper_link = robot_namespace + "/main_arm_SIM/gripper_manipulation_link";
    // move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_);
    // planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    // /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    // robot_model::RobotModelConstPtr robot_model_ = move_group_->getRobotModel();
    // robot_state::RobotStatePtr robot_state_(new robot_state::RobotState(robot_model_));
    // joint_model_group_ = robot_state_->getJointModelGroup(PLANNING_GROUP_);
// 
    // // planning_scene::PlanningScenePtr planning_scene_(new planning_scene::PlanningScene(robot_model_));
    // // robot_model_loader::RobotModelLoader robot_model_loader_("bvr_SIM/robot_description");
    // planning_scene_monitor_ = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>("bvr_SIM/robot_description");
    // planning_scene::PlanningScenePtr planning_scene_ = planning_scene_monitor_->getPlanningScene();
    // bool success = planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
    // ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));
    // //
    // planning_scene_monitor_->startSceneMonitor("move_group/monitored_planning_scene");
    // planning_scene_monitor_->startStateMonitor();
    // planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_STATE);
    // boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    // moveit_msgs::MotionPlanRequest req;
    // req.group_name = PLANNING_GROUP_;
    // moveit_msgs::MotionPlanResponse res;
    // planning_interface::PlannerManagerPtr planner_instance;
    // std::string planner_plugin_name;
// 
    // if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    // ROS_FATAL_STREAM("Could not find planner plugin name");
    // try
    // {
    //     planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
    //         "moveit_core", "planning_interface::PlannerManager"));
    // }
    // catch (pluginlib::PluginlibException& ex)
    // {
    //     ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    // }
    // try
    // {
    //     planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    //     if (!planner_instance->initialize(robot_model_, node_handle.getNamespace()))
    //     ROS_FATAL_STREAM("Could not initialize planner instance");
    //     ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    // }
    // catch (pluginlib::PluginlibException& ex)
    // {
    //     const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    //     std::stringstream ss;
    //     for (std::size_t i = 0; i < classes.size(); ++i)
    //     ss << classes[i] << " ";
    //     ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
    //                                                         << "Available plugins: " << ss.str());
    // }







    std::string robot_namespace = "bvr_SIM";
    std::string arm_namespace = "main_arm_SIM";

    tf_listener_ = std::make_unique<tf::TransformListener>();

    // Setup Visualization
    // visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(robot_namespace + "/bvr_base_link");
    // visual_tools_->deleteAllMarkers();
    // visual_tools_->trigger();

    // Setup Interface
    PLANNING_GROUP_              = arm_namespace;  //Assumes arm_namespace:=moveit planning group "bvr_SIM/main_arm_SIM"; 
    move_group_                  = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_);
    planning_scene_interface_    = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    // joint_model_group_           = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
    // const std::vector<std::string>& joint_model_group_names = move_group_->getJointModelGroupNames();
    // std::copy(joint_model_group_names.begin(), joint_model_group_names.end(), std::ostream_iterator<std::string>(std::cout, " "));

    // const moveit::core::JointModelGroup* joint_model_group = move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP_);

    robot_model::RobotModelConstPtr robot_model_ = move_group_->getRobotModel();
    robot_state::RobotStatePtr robot_state_(new robot_state::RobotState(robot_model_));
    joint_model_group_ = robot_state_->getJointModelGroup(PLANNING_GROUP_);

    // std::string plan_link_ = move_group_->getPlanningFrame();
    // std::string ee_link_   = robot_namespace + "/" + arm_namespace + "/gripper_manipulation_link";

    // std::string bvr_base_interia_frame = robot_namespace+"/bvr_base_inertia";
    // std::string gripper_link = robot_namespace + "/main_arm_SIM/gripper_manipulation_link";


    move_group_->setMaxVelocityScalingFactor(0.25);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    move_group_->setPlanningTime(5.0); //5.0
    move_group_->setNumPlanningAttempts(100); //10	
    move_group_->setGoalTolerance(0.02);
    move_group_->setPlannerId("RRTConnectkConfigDefault");


    // std::map<std::string,std::string> planner_params = move_group_->getPlannerParams(move_group_->getPlannerId(), PLANNING_GROUP_);










































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
                waypoint.push_back(xml_points_list[i][3]);
                waypoint.push_back(xml_points_list[i][4]);
                waypoint.push_back(xml_points_list[i][5]);
                waypoint.push_back(xml_points_list[i][6]);
                waypoint.push_back(xml_points_list[i][7]);
                waypoint.push_back(xml_points_list[i][8]);

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

    geometry_msgs::PoseStamped curr_pose = move_group_->getCurrentPose();
    move_base_msgs::MoveBaseGoal goal_base;
    geometry_msgs::PoseStamped goal_pose = curr_pose;

    // std::vector<double> prevMoveBase;

    for (const auto & line : waypoints_list) {

        goal_base.target_pose.pose.position.x = line[0];
        goal_base.target_pose.pose.position.y = line[1];
        // goal_base.target_pose.pose. (heading) line[2];
        // tf::Quaternion quat_b =  armToBaseOrientation(base_pose);
        // goal_base.target_pose.pose.orientation.x = quat_b.x();
        // goal_base.target_pose.pose.orientation.y = quat_b.y();
        // goal_base.target_pose.pose.orientation.z = quat_b.z();
        goal_base.target_pose.header.stamp = ros::Time::now();
        goal_base.target_pose.pose.orientation.w = 1.0f;
        goal_base.target_pose.header.frame_id = "map";
        ROS_INFO_STREAM("Moving Base to goal: "<< goal_base.target_pose.pose);
        // visual_tools.publishArrow(goal_base.target_pose.pose,rviz_visual_tools::colors::RED,rviz_visual_tools::scales::LARGE);
        // visual_tools.trigger();
        move_base(goal_base);


        // auto arm_pose = ArmBasePairs.arm_poses.poses[ind];
        // geometry_msgs::Pose
        // tf::Pose(tf::Quaternion(0,0,0,1), tf::Point(0,0,0))
        // geometry_msgs::Pose arm_pose = geometry_msgs::Pose
        // tf::Stamped<tf::Pose> arm_pose = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(0,0,0,1), tf::Point(1,0,0.5)), ros::Time::now(), world_link_);
        
        // geometry_msgs::PoseStamped arm_pose = curr_pose;

        // geometry_msgs::PoseStamped arm_pose = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(0,0,0,1), tf::Point(1,0,0.5)), ros::Time::now(), "map");

        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = "map";
        target_pose.header.stamp = ros::Time::now();
        target_pose.pose.position.x    = line[3]; 
        target_pose.pose.position.y    = line[4]; 
        target_pose.pose.position.z    = line[5]; 
        // target_pose.pose.orientation.x = line[6]; 
        // target_pose.pose.orientation.y = line[7]; 
        // target_pose.pose.orientation.z = line[8]; 
        target_pose.pose.orientation.w = 1.0f; // line[9]; 

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_->setStartState(*move_group_->getCurrentState());
        move_group_->setGoalOrientationTolerance(0.23);
        move_group_->setGoalPositionTolerance(0.05);
        ROS_INFO_STREAM("Attempting moving Arm to goal: "<< target_pose.pose);
        move_group_->setPoseTarget(target_pose); // goal_pose
        // visual_tools.publishArrow(goal_pose.pose,rviz_visual_tools::colors::YELLOW,rviz_visual_tools::scales::XLARGE);
        // visual_tools.trigger();
        // visual_tools.deleteAllMarkers();
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(success)
        {
            // ROS_INFO_STREAM("plan success for arm pose" << arm_pose);
            // visual_tools.prompt("Found solution Press 'next'to execute trajectory");
            // moveit::planning_interface::MoveItErrorCode exec_success = move_group_->execute(my_plan);
            
            move_group_->execute(my_plan);
            // move_group_->asyncExecute(my_plan);
            // ROS_INFO_STREAM("execution success message #"<< exec_success.val);
        }

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