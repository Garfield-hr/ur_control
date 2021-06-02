import math

from tf.transformations import quaternion_multiply, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from my_ur_planner import MyRobotPlanner, ControlMode
import rospy


class FeedbackPouringControl:

    # robot_initial_position: pose; geometry_parameter: [h, r]
    def __init__(self, robot, robot_init_pose, geometry_parameter):
        self.robot_init_pose = robot_init_pose
        self.robot_init_position = robot_init_pose.position
        self.robot_init_orientation = robot_init_pose.orientation
        self.max_height = geometry_parameter[0]
        self.cup_radius = geometry_parameter[1]
        self.angle = math.pi/6
        self.robot = robot

    def ratio_diff2height(self, ratio_diff):
        return 0.5*self.max_height*(ratio_diff + 1)

    def height2pose(self, height):
        expected_pose = Pose()

        init_ori = quaternion_format_transform(self.robot_init_orientation)
        trans = quaternion_from_euler(-self.angle ,0, 0)
        expected_pose.orientation = quaternion_format_transform(quaternion_multiply(init_ori, trans))

        expected_pose.position.x = self.robot_init_position.x
        expected_pose.position.y = self.robot_init_position.y - height*math.sin(self.angle) + \
                                   self.cup_radius*math.cos(self.angle)
        expected_pose.position.z = self.robot_init_position.z - height*math.sin(self.angle) + \
                                   self.cup_radius*math.sin(self.angle)
        return expected_pose

    def feedback_control(self, ratio):
        expected_ratio = 0.5
        ratio_diff = ratio - expected_ratio
        height = self.ratio_diff2height(ratio_diff)
        goal_pose = PoseStamped()
        goal_pose.header.seq = 1
        goal_pose.header.stamp = rospy.Time.from_sec(0.1)
        goal_pose.header.frame_id = 'my_planner'
        goal_pose.pose = self.height2pose(height)
        self.robot.control_using_ikfast(goal_pose)

    def switch_control(self, mode):
        if mode == 'violent':
            self.robot.control_using_ikfast(self.robot_init_pose)
        elif mode == 'slight':
            height = 0.05
            pose = self.height2pose(height)
            self.robot.control_using_ikfast(pose)
        else:
            print('no such mode')


def quaternion_format_transform(quat):
    if isinstance(quat, Quaternion):
        return [Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w]
    elif isinstance(quat, list):
        ret = Quaternion()
        [ret.x, ret.y, ret.z, ret.w] = quat
        return ret
    else:
        raise Exception('input type not supported, input type is {}'.format(type(quat)))

def get_command():
    ret = input('input command, \'violent\', \'slight\', \'quit\'')
    while ret not in ['violent', 'slight', 'quit']:
        print('invalid command, retry')
        ret = input('input command, \'violent\', \'slight\', \'quit\'')
    return ret

if __name__ == '__main__':
    simulation_flag = False
    if simulation_flag:
        topic_command = '/arm_controller/command'
        topic_state = '/arm_controller/state'
    else:
        topic_command = '/scaled_pos_traj_controller/command'
        topic_state = '/scaled_pos_traj_controller/state'
    control_mode = ControlMode.ikfast
    my_robot_planner = MyRobotPlanner(topic_command=topic_command,
                                      topic_state=topic_state, control_mode=control_mode)
    current_pose = my_robot_planner.robot.group.get_current_pose()
    pouring_control = FeedbackPouringControl(my_robot_planner, current_pose, [0.11, 0.03])
    print("robot ready to receive command")

    cmd =get_command()
    while cmd != 'quit':
        pouring_control.switch_control(cmd)

