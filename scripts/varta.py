#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

#---tutorial on ros-planning.github.io/moveit_tutorials---#

#---for moving---#
import rospy
import time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import math

import actionlib

#---for arm---#
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Transform (effector_camera -> Tray2) when gripper is centered above the tray
#center_x = -0.120
#center_y = -0.027
#center_z = 0.372
center_x = 0
center_y = 0
center_z = 0.372

center_orient_x = 0.004
center_orient_y = 0.998
center_orient_z = -0.019
center_orient_w = 0.060

# Z transform component (effector_camera -> Tray2) when gripper is touching tray
tray_target_z = 0.226

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("tdot_node")

moma = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

#---initiate MoveGroupCommander---#
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

#---Create DisplayTrajectory---#
display_trajectory_publisher = rospy.Publisher(
     "/move_group/display_planned_path",
     moveit_msgs.msg.DisplayTrajectory,
     queue_size=20,
)

# --- INITIALIZE TF ---
tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)

#---getting Basic Information---#
# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = moma.get_group_names()
print("============ Robot Groups:", moma.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(moma.get_current_state())
print("")


#---planning a joint goal---#
# We can get the joint values from the group and adjust some of the values:
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = -math.pi/2
joint_goal[1] = 0
joint_goal[2] = math.pi/2
joint_goal[3] = 0
joint_goal[4] = math.pi/2
joint_goal[5] = 0

# The go command can be called with joint values, poses, or without any
# parameters if you have already set the pose or joint target for the group
rospy.loginfo("Moving to home position")
success = move_group.go(joint_goal, wait=True)
if not success:
     rospy.logerr("Movement failed")
     exit(-1)

# Calling ``stop()`` ensures that there is no residual movement
move_group.stop()

# TODO
exit(0)

rospy.sleep(5)

trans = tfBuffer.lookup_transform('effector_camera', 'Tray2', rospy.Time())

offset_x = center_x - trans.transform.translation.x 
offset_y = center_y - trans.transform.translation.y 
offset_z = tray_target_z - trans.transform.translation.z 

offset_orient_x = center_orient_x - trans.transform.rotation.x 
offset_orient_y = center_orient_y - trans.transform.rotation.y 
offset_orient_z = center_orient_z - trans.transform.rotation.z 
offset_orient_w = center_orient_w - trans.transform.rotation.w 

waypoints = []

wpose = move_group.get_current_pose().pose
wpose.position.x -= offset_x
wpose.position.y -= offset_y
wpose.position.z += 0
#wpose.orientation.x += offset_orient_x
#wpose.orientation.y += offset_orient_y
#wpose.orientation.z += offset_orient_z
#wpose.orientation.w += offset_orient_w
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

rospy.loginfo("Moving above grip point")
move_group.execute(plan, wait=True)
move_group.stop()

rospy.sleep(5)

# TODO 
exit(0)

waypoints = []
wpose.position.z += offset_z 
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

move_group.execute(plan, wait=True)
move_group.stop()

# TODO 
exit(0)

client=actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server(rospy.Duration(20))

arm_client=actionlib.SimpleActionClient("robco_trajectory_controller/follow_joint_trajectory", 
FollowJointTrajectoryAction)
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
point_down.positions=[-1.5078012459935983, -0.3100678498656885, 
1.560142350442928, 0.0014141376675309882, 1.3614558856327104, 
1.2254828672671751]
arm_position2.trajectory.points.append(point_down)

#---Arm position while moving---#
arm_position_move=FollowJointTrajectoryGoal()
arm_position_move.trajectory.joint_names=["drive0_joint","drive1_joint","drive2_joint","drive3_joint","drive4_joint","drive5_joint"]

point_move=JointTrajectoryPoint()
point_move.time_from_start=rospy.Duration(5)
point_move.positions=[-1.510066265181798, -0.13164670913112816, 
1.5702690198927696, 0.0001318243693372813, 1.3609525459409377, 
1.2300009204676015]
arm_position_move.trajectory.points.append(point_move)



client.send_goal(table1)
client.wait_for_result()

arm_client.send_goal(arm_position2)
arm_client.wait_for_result()

time.sleep(5)

arm_client.send_goal(arm_position_move)
arm_client.wait_for_result()

client.send_goal(table2)
client.wait_for_result()

arm_client.send_goal(arm_position2)
arm_client.wait_for_result()

time.sleep(5)

arm_client.send_goal(arm_position_move)
arm_client.wait_for_result()
