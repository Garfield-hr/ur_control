import VisualFeedback.visual_feedback as vf
from ximea import xiapi
import cv2 as cv
from pouring_feedback import FeedbackPouringControl, ControlMode, MyRobotPlanner
from VisualFeedback.camera_setting import white_balance_adjustment
import rospy


def func1():
    # robot setting
    def robot_setting():
        rospy.init_node('pouring_control', anonymous=True, disable_signals=True)
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
        current_pose = my_robot_planner.robot.group.get_current_pose().pose
        pouring_control = FeedbackPouringControl(my_robot_planner, current_pose, [0.11, 0.03])
        print("robot ready to receive command")
        return pouring_control
    pouring_control = robot_setting()

    # camera setting
    def camera_setting():
        cam = xiapi.Camera()
        # start communication
        print('Opening first camera...')
        cam.open_device()

        # settings
        cam.set_imgdataformat('XI_RGB24')
        # cam.set_exposure(792)
        # cam.set_region_selector(0)
        # cam.set_width(1264)
        # cam.set_height(1016)
        # cam.set_gain(15)

        # create instance of Image to store image data and metadata
        img = xiapi.Image()

        # start data acquisition
        print('Starting data acquisition...')

        # cam.start_acquisition()

        def adjust_camera():
            image = xiapi.Image()
            while cv.waitKey(33) != 27:
                cam.get_image(img)
                data = image.get_image_data_numpy()
                cv.imshow("img", data)

        white_balance_adjustment(cam)
        print('please adjust camera for pouring measurement')
        adjust_camera(cam)
        return cam
    cam = camera_setting()

    def read_img():
        image = cam.get_image()
        image_np = image.get_image_data_numpy()
        return True, image_np
    cam_roi = vf.RoiByFourPoints(read_img)

    n = 10
    while True:
        try:
            ratio = 0
            for i in range(n):
                ratio += cam_roi.get_foam_ratio()
            ratio /= n
            liquid_level = pouring_control.max_height*cam_roi.get_liquid_level()
            pouring_control.auto_switch_control(ratio, liquid_level=liquid_level)
        except KeyboardInterrupt as e:
            break


def ball_pouring_control():
    # change to slight mode when ball full half of the cup
    # robot setting
    def robot_setting():
        rospy.init_node('pouring_control', anonymous=True, disable_signals=True)
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
        current_pose = my_robot_planner.robot.group.get_current_pose().pose
        pouring_control = FeedbackPouringControl(my_robot_planner, current_pose, [0.11, 0.03])
        print("robot ready to receive command")
        return pouring_control

    pouring_control = robot_setting()

    # camera setting
    def camera_setting():
        cam = xiapi.Camera()
        # start communication
        print('Opening first camera...')
        cam.open_device()

        # settings
        cam.set_imgdataformat('XI_RGB24')
        # cam.set_exposure(792)
        # cam.set_region_selector(0)
        # cam.set_width(1264)
        # cam.set_height(1016)
        # cam.set_gain(15)

        # create instance of Image to store image data and metadata
        img = xiapi.Image()

        # start data acquisition
        print('Starting data acquisition...')

        # cam.start_acquisition()

        def adjust_camera():
            image = xiapi.Image()
            while cv.waitKey(33) != 27:
                cam.get_image(img)
                data = image.get_image_data_numpy()
                cv.imshow("img", data)

        white_balance_adjustment(cam)
        print('please adjust camera for pouring measurement')
        adjust_camera(cam)
        return cam

    cam = camera_setting()

    def read_img():
        image = cam.get_image()
        image_np = image.get_image_data_numpy()
        return True, image_np

    cam_roi = vf.RoiByFourPoints(read_img)

    n = 10
    while True:
        try:
            ratio = 0
            for i in range(n):
                ratio += cam_roi.get_foam_ratio()
            ratio /= n
            liquid_level = pouring_control.max_height * cam_roi.get_liquid_level()
            pouring_control.ball_control(liquid_level)
        except KeyboardInterrupt as e:
            break




if __name__ == '__main__':
    pass