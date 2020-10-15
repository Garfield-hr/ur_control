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

    def __init__(self, topic_command='/arm_controller/command', topic_state='/arm_controller/state', moveit_flag=False):
        self.robot = MoveGroupPythonInteface()
        self.pub = rospy.Publisher(topic_command, JointTrajectory, queue_size=10)
        self.pub_map = rospy.Publisher('joint1_state', Float64, queue_size=10)
        self.robot_monitor = RobotStateMonitor()
        self.use_moveit_plan = moveit_flag
        rospy.Subscriber(topic_state, JointTrajectoryControllerState, self.robot_monitor.update_robot_state)
        print('robot go home')
        self.robot.go_home()

    def control_robot(self, goal_point):
        if not self.use_moveit_plan:
            self.control_robot_without_moveit(goal_point)
        else:
            self.control_robot_with_moveit(goal_point)

    def control_robot_without_moveit(self, goal_point):
        start_point = self.robot_monitor.joint_point
        complete_point(start_point)
        start_point.time_from_start = rospy.Duration.from_sec(0)
        complete_point(goal_point)
        goal_point.positions = nearer_position(start_point.positions, goal_point.positions)
        # print(goal_point)
        traj, _ = traj_generate_with_two_points(start_point, goal_point)
        self.pub.publish(traj)
        self.pub_map.publish(self.robot_monitor.joint_point.positions[0])

    def control_robot_with_moveit(self, goal_point):
        start_point = self.robot_monitor.joint_point
        complete_point(start_point)
        start_point.time_from_start = rospy.Duration.from_sec(0)
        complete_point(goal_point)
        goal_point.positions = nearer_position(start_point.positions, goal_point.positions)
        self.robot.group.set_joint_value_target(goal_point.positions)
        plan = self.robot.group.plan()
        self.modify_time(plan.joint_trajectory.points)
        self.pub.publish(plan.joint_trajectory)

    def modify_time(self, points, expected_time):
        if not points:
            print("empty points")
            exit(1)
        previous_last_time = points[-1].time_from_start.to_sec()
        for i in range(len(points)):
            new_time = expected_time*points[i].time_from_start.to_sec()/previous_last_time
            points[i].time_from_start = new_time


def complete_point(point):
    if not len(point.positions):
        point.positions = [0, 0, 0, 0, 0, 0]
    if not len(point.velocities):
        point.velocities = [0, 0, 0, 0, 0, 0]
    if not len(point.accelerations):
        point.accelerations = [0, 0, 0, 0, 0, 0]


def nearer_position(start_position, goal_position):
    result_position = []
    equivalent_position = lambda x: x - 2 * math.pi if x > 0 else x + 2 * math.pi
    nearer_position_between_two_points = lambda x, a, b: a if abs(x - a) < abs(x - b) else b
    for (spi, gpi) in zip(start_position, goal_position):
        result_position.append(nearer_position_between_two_points(spi, gpi, equivalent_position(gpi)))

    return result_position


if __name__ == '__main__':
    rospy.init_node('my_controller', anonymous=True)
    simulation = True
    if simulation:
        topic_command = '/arm_controller/command'
        topic_state = '/arm_controller/state'
    else:
        topic_command = '/scaled_pos_traj_controller/command'
        topic_state = 'scaled_pos_traj_controller/state'
    my_robot_planner = MyRobotPlanner(topic_command=topic_command, topic_state=topic_state)
    use_moveit_plan = False
    rospy.Subscriber('my_command', JointTrajectoryPoint, my_robot_planner.control_robot,use_moveit_plan)

    rospy.spin()

