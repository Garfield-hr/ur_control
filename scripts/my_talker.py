#!/usr/bin/env python2

import rospy, math
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
from geometry_msgs.msg import Pose

def talker():
    rospy.init_node('talker', anonymous=True)
    #pub = rospy.Publisher('my_command_pose', JointTrajectoryPoint, queue_size=10)
    pub = rospy.Publisher('my_command_pose', Pose, queue_size=10)
    rate = rospy.Rate(10)
    goal_pose = Pose()
    goal_pose.position = geometry_msgs.msg.Point(x=0.3, y=0, z=0.9)
    goal_pose.orientation = geometry_msgs.msg.Quaternion(x=0, y=-math.sin(math.pi/4), z=0, w=math.cos(math.pi/4))
    seq = 1
    while not rospy.is_shutdown():
        goal_pose.position.x = 0.4*math.cos(0.1*seq)
        goal_pose.position.y = 0.4*math.sin(0.1*seq)
        seq += 1
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