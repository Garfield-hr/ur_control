#!/usr/bin/env python2
from quadrillion_test import *
from moveit_test import MoveGroupPythonInteface
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64


class RobotStateMonitor:

    def __init__(self):
        self.joint_point = JointTrajectoryPoint()

    def update_robot_state(self, data):
        self.joint_point = data.actual


class MyRobotPlanner:

    def __init__(self, topic_command='/arm_controller/command', topic_state='/arm_controller/state'):
        self.robot = MoveGroupPythonInteface()
        self.pub = rospy.Publisher(topic_command, JointTrajectory, queue_size=10)
        self.pub_map = rospy.Publisher('joint1_state', Float64, queue_size=10)
        self.robot_monitor = RobotStateMonitor()
        rospy.Subscriber(topic_state, JointTrajectoryControllerState, self.robot_monitor.update_robot_state)
        print('robot go home')
        self.robot.go_home()

    def control_robot(self, goal_point):
        start_point = self.robot_monitor.joint_point
        complete_point(start_point)
        start_point.time_from_start = rospy.Duration.from_sec(0)
        complete_point(goal_point)
        #print(goal_point)
        traj, _ = traj_generate_with_two_points(start_point, goal_point)
        self.pub.publish(traj)
        self.pub_map.publish(self.robot_monitor.joint_point.positions[0])


def complete_point(point):
    if not len(point.positions):
        point.positions = [0, 0, 0, 0, 0, 0]
    if not len(point.velocities):
        point.velocities = [0, 0, 0, 0, 0, 0]
    if not len(point.accelerations):
        point.accelerations = [0, 0, 0, 0, 0, 0]


if __name__ == '__main__':
    rospy.init_node('my_controller', anonymous=True)
    my_robot_planner = MyRobotPlanner()
    rospy.Subscriber('my_command', JointTrajectoryPoint, my_robot_planner.control_robot)

    rospy.spin()

