#!/usr/bin/env python

# just for overleaf NOT TESTED

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

center_x = 0
center_y = 0
center_z = 0.372

center_orient_x = 0.004
center_orient_y = 0.998
center_orient_z = -0.019
center_orient_w = 0.060

# --- INITIALIZE TF ---
tfBuffer = tf2_ros.Buffer()
tfListener = tf2_ros.TransformListener(tfBuffer)

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