import math
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from RobotControl.my_ur_planner import MyRobotPlanner, ControlMode, UrControl
import rospy
from copy import deepcopy


class FeedbackPouringControl:

    # robot_initial_position: pose; geometry_parameter: [h, r]
    # ratio is beer to all and become larger in slight mode
    def __init__(self, robot, geometry_parameter, assembly_height=0.05):
        self.max_height = geometry_parameter[0]
        self.cup_radius = geometry_parameter[1]
        self.angle = math.pi/8
        self.robot = robot
        self.mode = 'violent'
        self.expected_ratio = 0.4
        self.allowable_error = 0.1
        self.assembly_height = assembly_height
        self.go_home()
        self.error_sum = 0
        self.error_bef = 0
        self.height = 0

    def go_home(self):
        goal_pose = PoseStamped()
        goal_pose.header.seq = 1
        goal_pose.header.stamp = rospy.Time.from_sec(5)
        goal_pose.header.frame_id = 'my_planner'

        goal_pose.pose.position.x = 0.466
        goal_pose.pose.position.y = 0.1
        goal_pose.pose.position.z = 0.536
        goal_pose.pose.orientation.x = 0
        goal_pose.pose.orientation.y = 0
        goal_pose.pose.orientation.z = -math.sin(math.pi / 4)
        goal_pose.pose.orientation.w = math.cos(math.pi / 4)
        self.robot.go_to_pose(goal_pose)
        self.robot_init_pose = goal_pose.pose
        self.robot_init_position = self.robot_init_pose.position
        self.robot_init_orientation = self.robot_init_pose.orientation
        print('reach home')
        height = 0.05
        pose = self.height2pose(height)
        goal_pose = PoseStamped()
        goal_pose.header.seq = 1
        goal_pose.header.stamp = rospy.Time.from_sec(0.5)
        goal_pose.header.frame_id = 'my_planner'
        goal_pose.pose = pose
        self.mode = 'slight'
        self.robot.go_to_pose(goal_pose)

    def ratio_diff2height(self, ratio_diff):
        return 0.5*self.max_height*(ratio_diff + 1)

    def height2pose(self, height):
        expected_pose = Pose()

        init_ori = quaternion_format_transform(self.robot_init_orientation)
        trans = quaternion_from_euler(0, -self.angle, 0)

        expected_pose.orientation = quaternion_format_transform(quaternion_multiply(init_ori, trans))

        expected_pose.position.x = self.robot_init_position.x
        expected_pose.position.y = self.robot_init_position.y + self.assembly_height * math.sin(self.angle) - height*math.sin(self.angle) - \
                                   self.cup_radius*math.cos(self.angle)
        expected_pose.position.z = self.robot_init_position.z - self.assembly_height * (1 - math.cos(self.angle)) - height*math.sin(self.angle) + \
                                   self.cup_radius*math.sin(self.angle)
        # expected_pose.position.y = self.robot_init_position.y + 0.05 * math.sin(self.angle)
        # expected_pose.position.z = self.robot_init_position.z - 0.05 * (1 - math.cos(self.angle))
        return expected_pose

    def feedback_control(self, ratio):
        expected_ratio = 0.6
        ratio_diff = ratio - expected_ratio
        height = self.ratio_diff2height(ratio_diff)
        goal_pose = PoseStamped()
        goal_pose.header.seq = 1
        goal_pose.header.stamp = rospy.Time.from_sec(0.1)
        goal_pose.header.frame_id = 'my_planner'
        goal_pose.pose = self.height2pose(height)
        self.robot.go_to_pose(goal_pose)

    def switch_control(self, mode):
        if mode == 'violent':
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(3)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = self.robot_init_pose
            self.robot.go_to_pose(goal_pose)
            self.mode = 'violent'
        elif mode == 'slight':
            height = 0.08
            pose = self.height2pose(height)
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(3)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = self.robot_init_pose
            goal_pose.pose = pose
            self.robot.go_to_pose(goal_pose)
            self.mode = 'slight'
        elif mode == 'high':
            height = 0.08
            pose = self.height2pose(height)
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(3)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = self.robot_init_pose
            goal_pose.pose = pose
            goal_pose.pose.position.z += 0.1
            self.robot.go_to_pose(goal_pose)
            self.mode = 'high'
        else:
            print('no such mode')

    def auto_switch_control(self, ratio, liquid_level=0):
        if self.mode == 'violent' and ratio < (1 - self.allowable_error)*self.expected_ratio:
            # too much foam, switch to slight mode
            height = min(0.09, 0.02+liquid_level)
            pose = self.height2pose(height)
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = pose
            self.robot.go_to_pose(goal_pose)
            self.mode = 'slight'
        elif self.mode == 'slight' and ratio > (1 + self.allowable_error)*self.expected_ratio:
            # too little foam, switch to violent mode
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = self.robot_init_pose
            self.robot.go_to_pose(goal_pose)
            self.mode = 'violent'

    def ball_control(self, liquid_level):
        if self.mode == 'slight':
            return
        if liquid_level > 0.03:
            height = min(0.09, 0.02 + liquid_level)
            pose = self.height2pose(height)
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = self.robot_init_pose
            goal_pose.pose = pose
            self.robot.go_to_pose(pose)
            self.mode = 'slight'

    def ball_control_pid(self, liquid_level):
        if liquid_level > 0.03 or self.mode == 'slight':
            height = min(0.09, 0.02 + liquid_level)
            pose = self.height2pose(height)
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = pose
            self.mode = 'slight'
        else:
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = deepcopy(self.robot_init_pose)
            self.mode = 'violent'
        delta_height = liquid_level
        goal_pose.pose.position.z += delta_height
        self.robot.go_to_pose(goal_pose)

    def beer_pid_backup(self, ratio, liquid_level=0):
        error = self.expected_ratio - ratio
        # pid = max(0, 2*error + 0.07*self.error_sum)
        # pid = min(pid, 1)
        error_diff = error - self.error_bef
        pid = 1*error + 0.2*error_diff
        delta_height = 0.8*pid*self.max_height
        self.error_sum += error
        if error < -self.allowable_error or (((error > -self.allowable_error) and (error < self.allowable_error)) and self.mode == 'violent'):
            # two much beer or used to be using violent and it's ok to keep, use violent
            # goal_pose = PoseStamped()
            # goal_pose.header.seq = 1
            # goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            # goal_pose.header.frame_id = 'my_planner'
            # goal_pose.pose = deepcopy(self.robot_init_pose)
            # self.mode = 'violent'
            height = min(0.09, 0.05 + liquid_level)
            pose = self.height2pose(height)
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = pose
            self.mode = 'slight'
        else:
            height = min(0.09, 0.05 + liquid_level)
            pose = self.height2pose(height)
            goal_pose = PoseStamped()
            goal_pose.header.seq = 1
            goal_pose.header.stamp = rospy.Time.from_sec(0.5)
            goal_pose.header.frame_id = 'my_planner'
            goal_pose.pose = pose
            self.mode = 'slight'
            self.height += delta_height
            self.height = min(self.height, 0.8*self.max_height)
            self.height = max(0, self.height)
            goal_pose.pose.position.z += self.height
        self.robot.go_to_pose(goal_pose)
        return self.height

    def beer_control_pid(self, ratio, liquid_level=0):
        error = self.expected_ratio - ratio
        # pid = max(0, 2*error + 0.07*self.error_sum)
        # pid = min(pid, 1)
        error_diff = error - self.error_bef
        pid = 2*error + 0.3*error_diff
        delta_height = pid*self.max_height
        self.error_sum += error
        height = min(0.12, 0.08 + liquid_level)
        pose = self.height2pose(height)
        goal_pose = PoseStamped()
        goal_pose.header.seq = 1
        goal_pose.header.stamp = rospy.Time.from_sec(0.5)
        goal_pose.header.frame_id = 'my_planner'
        goal_pose.pose = pose
        self.mode = 'slight'
        self.height += delta_height
        self.height = min(self.height, 0.8 * self.max_height)
        self.height = max(0, self.height)
        goal_pose.pose.position.z += self.height

        self.robot.go_to_pose(goal_pose, wait=False)
        return self.height


def quaternion_format_transform(quat):
    if isinstance(quat, Quaternion):
        return [quat.x, quat.y, quat.z, quat.w]
    elif isinstance(quat, (np.ndarray, list)):
        ret = Quaternion()
        [ret.x, ret.y, ret.z, ret.w] = quat
        return ret
    else:
        raise Exception('input type not supported, input type is ', type(quat))


def get_command():
    ret = input('input command, \'violent\', \'slight\', \'quit\'')
    while ret not in ['violent', 'slight', 'quit']:
        print('invalid command, retry')
        ret = input('input command, \'violent\', \'slight\', \'quit\'')
    return ret


if __name__ == '__main__':
    rospy.init_node('pouring_control', anonymous=True, disable_signals=True)
    # simulation_flag = False
    # if simulation_flag:
    #     topic_command = '/arm_controller/command'
    #     topic_state = '/arm_controller/state'
    # else:
    #     topic_command = '/scaled_pos_traj_controller/command'
    #     topic_state = '/scaled_pos_traj_controller/state'
    # control_mode = ControlMode.ikfast
    # my_robot_planner = MyRobotPlanner(topic_command=topic_command,
    #                                   topic_state=topic_state, control_mode=control_mode)
    # current_pose = my_robot_planner.robot.group.get_current_pose().pose
    robot = UrControl()
    pouring_control = FeedbackPouringControl(robot, [0.11, 0.03])
    print("robot ready to receive command")

    while True:
        pouring_control.switch_control('slight')
        pouring_control.switch_control('violent')

