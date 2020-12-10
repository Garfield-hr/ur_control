#!/usr/bin/env python2

import rospy, math
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def talker():
    rospy.init_node('talker', anonymous=True)
    #pub = rospy.Publisher('my_command_pose', JointTrajectoryPoint, queue_size=10)
    pub = rospy.Publisher('catch_point', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)
    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.get_rostime()
    goal_pose.pose.position = geometry_msgs.msg.Point(x=-0.37, y=-0.24, z=0.71)
    theta = 0
    goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(x=1*math.sin(theta/2), y=0, z=0, w=math.cos(theta/2))
    for i in range(10):
        theta += 0.05*math.pi
        goal_pose.pose.position.x += 0.05
        goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(x=1 * math.sin(theta / 2), y=0, z=0,
                                                                  w=math.cos(theta / 2))
        goal_pose.header.stamp = rospy.get_rostime() + rospy.Duration.from_sec(1 - 0.1*i)

        pub.publish(goal_pose)
        rate.sleep()
    # gaol_point = JointTrajectoryPoint()
    # gaol_point.positions = [-0.4, -1.8, 1.8, 0, 1.2, 3.1]
    # gaol_point.time_from_start = rospy.Duration.from_sec(0.2)
    # seq = 1
    # while not rospy.is_shutdown():
    #     gaol_point.positions[0] += math.sin(seq)
    #     seq += 1
    #     pub.publish(gaol_point)
    #     rate.sleep()


if __name__ == '__main__':
    talker()