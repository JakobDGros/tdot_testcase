#!/usr/bin/env python

import rospy
import time

rospy.init_node("testnode")

while True:
    print("begin counting")
    for i in range (10):
        print(i)
        time.sleep(1)

    if rospy.is_shutdown():
        print("shutdown")
        break
    
    print("spin finished")
    print("finished")