#!/usr/bin/env python2

from moveit_test import MoveGroupPythonInteface
import rospy
import random
import copy
import time

rospy.init_node('robot_calibration', anonymous=True)
robot = MoveGroupPythonInteface()
robot.go_home()
print("please set up the robot, Press any key to continue")
input_key = raw_input()
start_pose_stamped = robot.group.get_current_pose()
try:
    for i in range(10):
        next_pose = copy.deepcopy(start_pose_stamped.pose)
        next_pose.position.x += random.random() * 0.1
        next_pose.position.y += random.random() * 0.1
        next_pose.position.z += random.random() * 0.1
        print(next_pose)
        robot.go_to_pose(next_pose)
        time.sleep(2)

except KeyboardInterrupt:
    print("keyboard interrupt")
