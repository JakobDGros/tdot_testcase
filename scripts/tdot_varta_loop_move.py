#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionFeedback, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal


import actionlib

rospy.init_node("tdot_node")

client=actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server(rospy.Duration(20))


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


while True:
    client.send_goal(table1)
    client.wait_for_result()

    if rospy.is_shutdown():
        print("shutdown")
        break    

    time.sleep(5)

    client.send_goal(table2)
    client.wait_for_result()

    if rospy.is_shutdown():
        print("shutdown")
        break

    time.sleep(5)

print("FINISHED")