#!/usr/bin/env python

# This file is mostly a redress of the base_move_to_goal.py file provided by Prateek Arora, from his 
#   siplanner project archive that he provided as reference. I started toward trying to make a similar 
#   prototype in the boat_planner /src folder, but he reccomended just sticking with Python for the 
#   prototype efforts. He also suggested http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals 

import time
import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import *
from trajectory_msgs.msg import *


hosting_robot_prefix = "bvr_SIM/"
ur5_e_robot_prefix = "main_arm_SIM/"
ur5e_arm_prefix = hosting_robot_prefix + ur5_e_robot_prefix
ur5e_arm_controller_topic = ur5_e_robot_prefix + "velocity_controller/follow_joint_trajectory"

JOINT_NAMES = [ur5e_arm_prefix+'shoulder_pan_joint',
               ur5e_arm_prefix+'shoulder_lift_joint',
               ur5e_arm_prefix+'elbow_joint',
               ur5e_arm_prefix+'wrist_1_joint',
               ur5e_arm_prefix+'wrist_2_joint',
               ur5e_arm_prefix+'wrist_3_joint']

shoulder_pan_home = 0.0 #1.0*math.pi/2
shoulder_lift_home = -1.5*math.pi/6
elbow_home = -4.5*math.pi/6
wrist_1_home = -0.15*math.pi/2
wrist_2_home = 1.0*math.pi/2
wrist_3_home = 0.0*math.pi/2

Qhome = [shoulder_pan_home                    ,shoulder_lift_home                     ,elbow_home                     ,wrist_1_home                    ,wrist_2_home                    ,wrist_3_home]
slowdown = 1.0

# client = None


# Prime Answer: 
# https://answers.ros.org/question/252236/what-is-the-recommended-way-to-give-a-robot-a-predefined-set-of-waypoints/

# Descendants:
# http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
# https://answers.ros.org/question/244175/sending-multiple-goals-to-navigation-stack/
# https://github.com/cristian-frincu/simple_navigation_goals
# https://answers.ros.org/question/373570/how-to-send-waypoints-to-move_base-using-rospy/
# https://answers.ros.org/question/361607/how-to-save-wavepoints/
# http://wiki.ros.org/follow_waypoints



def heading_to_q(theta):
    v = [0,0,1] # normalize(v)
    x, y, z = v
    theta /= 2
    x = x * math.sin(theta)
    y = y * math.sin(theta)
    z = z * math.sin(theta)
    w = math.cos(theta)
    return x, y, z, w


def movebase_client():
    client = actionlib.SimpleActionClient('bvr_SIM/move_base',MoveBaseAction)

# Needs to be a YAML file, for later
    waypoints = [
        # Run around the boat CCW
        [0,   0, 0, 0],
        [1,  -1, 0, 0.78540],
        [2,-1.1, 0, 1.57080],
        [3,-1.1, 0, 1.57080],
        [4,-1.1, 0, 1.57080],
        [5,-1.1, 0, 1.57080],
        [5.8, 0, 0, 3.14159],
        [5, 1.1, 0,-1.57080],
        [4, 1.1, 0,-1.57080],
        [3, 1.1, 0,-1.57080],
        [2, 1.1, 0,-1.57080],
        [1,   1, 0,-0.78540],
        [1.2, 0, 0, 0]]
        # {x:0,  y: 0, z:0, w:1},
        # {x:1,  y:-1, z:0, w:1},
        # {x:2,  y:-1, z:0, w:1},
        # {x:3,  y:-3, z:0, w:1},
        # {x:4,  y:-3, z:0, w:1},
        # {x:5,  y:-2, z:0, w:1},
        # {x:5.5,y: 0, z:0, w:1},
        # {x:5,  y: 2, z:0, w:1}]


    for pnt in waypoints:
        client.wait_for_server()
        print("wait_for_server")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pnt[0]
        goal.target_pose.pose.position.y = pnt[1]
        quat = heading_to_q(pnt[3])
        goal.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(quat[0],quat[1],quat[2],quat[3])
        # goal.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(0,0,-0.707,0.707)

        client.send_goal(goal)
        print("send_goal")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        # else:
        #     return client.get_result()   



def move_arm_home():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    g.trajectory.points = [
        JointTrajectoryPoint(positions=Qhome, velocities=[0]*6, time_from_start=rospy.Duration(slowdown*3.0))]
    client.send_goal(g)
    try:
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    global client
    try:
        rospy.init_node('movement_test')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise