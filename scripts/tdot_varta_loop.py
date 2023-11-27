#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


import actionlib

rospy.init_node("tdot_node")

client=actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server(rospy.Duration(20))

arm_client=actionlib.SimpleActionClient("robco_trajectory_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
arm_client.wait_for_server(rospy.Duration(20))

table1=MoveBaseGoal()
table1.target_pose.header.frame_id="map"
table1.target_pose.header.stamp=rospy.Time.now()
table1.target_pose.pose.position.x=3.3578
table1.target_pose.pose.position.y=2.2618
table1.target_pose.pose.position.z
table1.target_pose.pose.orientation.x=0.0
table1.target_pose.pose.orientation.y=0.0
table1.target_pose.pose.orientation.z=0.1138
table1.target_pose.pose.orientation.w=0.9935


table2=MoveBaseGoal()
table2.target_pose.header.frame_id="map"
table2.target_pose.header.stamp=rospy.Time.now()
table2.target_pose.pose.position.x=1.0249
table2.target_pose.pose.position.y=1.7228
table2.target_pose.pose.position.z
table2.target_pose.pose.orientation.x=0.0
table2.target_pose.pose.orientation.y=0.0
table2.target_pose.pose.orientation.z=-0.61937
table2.target_pose.pose.orientation.w=0.7851

#---Arm position while picking up---#
arm_position2=FollowJointTrajectoryGoal()
arm_position2.trajectory.joint_names=["drive0_joint","drive1_joint","drive2_joint","drive3_joint","drive4_joint","drive5_joint"]

point_down=JointTrajectoryPoint()
point_down.time_from_start=rospy.Duration(5)
point_down.positions=[-1.5078012459935983, -0.3100678498656885, 1.560142350442928, 0.0014141376675309882, 1.3614558856327104, 1.2254828672671751]
arm_position2.trajectory.points.append(point_down)

#---Arm position while moving---#
arm_position_move=FollowJointTrajectoryGoal()
arm_position_move.trajectory.joint_names=["drive0_joint","drive1_joint","drive2_joint","drive3_joint","drive4_joint","drive5_joint"]

point_move=JointTrajectoryPoint()
point_move.time_from_start=rospy.Duration(5)
point_move.positions=[-1.510066265181798, -0.13164670913112816, 1.5702690198927696, 0.0001318243693372813, 1.3609525459409377, 1.2300009204676015]
arm_position_move.trajectory.points.append(point_move)


while True:
    client.send_goal(table1)
    client.wait_for_result()

    if rospy.is_shutdown():
        print("shutdown")
        break    

    arm_client.send_goal(arm_position2)
    arm_client.wait_for_result()

    time.sleep(5)

    if rospy.is_shutdown():
        print("shutdown")
        break

    arm_client.send_goal(arm_position_move)
    arm_client.wait_for_result()

    client.send_goal(table2)
    client.wait_for_result()

    if rospy.is_shutdown():
        print("shutdown")
        break

    arm_client.send_goal(arm_position2)
    arm_client.wait_for_result()

    time.sleep(5)

    if rospy.is_shutdown():
        print("shutdown")
        break

    arm_client.send_goal(arm_position_move)
    arm_client.wait_for_result()

    if rospy.is_shutdown():
        print("shutdown")
        break

print("FINISHED")