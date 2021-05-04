#ifndef MGI_BOATPLANNER_
#define MGI_BOATPLANNER_

#include <moveit/move_group/node_name.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sensor_msgs/Joy.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


namespace move_group_interface {

class MGI_BoatPlanner {
  public:
    MGI_BoatPlanner(ros::NodeHandle nh, ros::NodeHandle private_nh, std::string world_link="map")
      : nh_(nh)
      , private_nh_(private_nh)
      , world_link_(world_link)
    {
      initialize();
    }

  private:
    void initialize();    

    void arm_reach();    

    // void rc_cb(const sensor_msgs::Joy::ConstPtr& msg);
    // void targetpose_2d_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    // void targetpoint_3d_cb(const geometry_msgs::PointStamped::ConstPtr& msg);
    void hardcoded_3d_cb();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    const std::string world_link_;
    std::string plan_link_;
    std::string ee_link_;

    std::string PLANNING_GROUP_;

    std::map<std::string, tf::StampedTransform> object_map_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<tf::TransformListener> tf_listener_;

    const robot_state::JointModelGroup* joint_model_group_;

    tf::StampedTransform T_EE_W_;  // end-effector to world
    tf::Stamped<tf::Pose> T_EE_ref_W_{tf::Pose(), ros::Time(0), ""};  // end-effector-reference to world

    // ros::Subscriber rc_sub_;
    // ros::Subscriber targetpose_2d_sub_;
    // ros::Subscriber targetpoint_3d_sub_;

  
};  //  class MGI_BoatPlanner

}  // namespace move_group_interface

#endif  // MGI_BOATPLANNER_
