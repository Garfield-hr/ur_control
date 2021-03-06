import matplotlib.pyplot as plt

import VisualFeedback.visual_feedback as vf
from ximea import xiapi
import cv2 as cv
from RobotControl.pouring_feedback import FeedbackPouringControl, ControlMode, MyRobotPlanner, UrControl
from VisualFeedback.camera_setting import white_balance_adjustment
import rospy
import thread
import time


class VisualMonitor:

    def __init__(self):
        cam = camera_setting()
        self.__ratio_sum = 0.0
        self.__ratio_count = 0
        self.__liquid_level = 0.0
        self.time_list = []
        self.ratio_updated = False
        self.ave_filter_size = 10
        self.__ratio = 0.0
        self.__ratio_updated = False

        def read_img():
            image = xiapi.Image()
            cam.get_image(image)
            image_np = image.get_image_data_numpy()
            return True, image_np

        self.cam_roi = vf.RoiByFourPoints(read_img)
        self.__stop = False

    def monitor_pouring(self):
        while not self.__stop:
            time_start = time.time()
            ratio = self.cam_roi.get_foam_ratio(debug=True)
            if not self.cam_roi.started:
                continue
            self.__ratio_sum += ratio
            self.__ratio_count += 1
            if self.__ratio_count == self.ave_filter_size:
                self.__ratio = 0.1 * self.__ratio_sum
                self.__ratio_updated = True
                self.__ratio_sum = 0.0
                self.__ratio_count = 0

            self.__liquid_level = self.cam_roi.get_liquid_level()
            self.time_list.append(1000*(time.time() - time_start))

    def get_ratio(self):
        if self.__ratio_updated:
            self.__ratio_updated = False
            return True, self.__ratio
        else:
            return False, False

    def stop_monitoring(self):
        self.__stop = True

    def get_liquid_level(self):
        return self.__liquid_level

    def test_tracking(self):
        while not self.__stop:
            _, img = self.cam_roi.get_roi_img()
            cv.imshow('img', img)
            cv.waitKey(1)


def camera_setting():
    cam = xiapi.Camera()
    # start communication
    print('Opening first camera...')
    cam.open_device()

    # settings
    cam.set_imgdataformat('XI_RGB24')
    cam.set_exposure(792)
    cam.set_region_selector(0)
    # cam.set_width(400)
    # cam.set_height(640)
    cam.set_gain(15)

    # create instance of Image to store image data and metadata
    img = xiapi.Image()

    # start data acquisition
    print('Starting data acquisition...')

    cam.start_acquisition()

    def adjust_camera():
        image = xiapi.Image()
        while cv.waitKey(33) != 27:
            cam.get_image(image)
            data = image.get_image_data_numpy()
            cv.imshow("img", data)

    white_balance_adjustment(cam)
    print('please adjust camera for pouring measurement')
    adjust_camera()
    cv.destroyAllWindows()
    return cam


def robot_setting():
    robot = UrControl()
    pouring_control = FeedbackPouringControl(robot, [0.14, 0.03])
    print("robot ready to receive command")
    return pouring_control


def beer_switch_control():
    # robot setting
    pouring_control = robot_setting()

    # camera setting
    cam = camera_setting()

    def read_img():
        image = xiapi.Image()
        cam.get_image(image)
        image_np = image.get_image_data_numpy()
        return True, image_np

    cam_roi = vf.RoiByFourPoints(read_img)

    n = 10
    while True:
        try:
            ratio = 0
            for i in range(n):
                ratio += cam_roi.get_foam_ratio(debug=True)
            ratio /= n
            liquid_level = pouring_control.max_height * cam_roi.get_liquid_level()
            pouring_control.auto_switch_control(ratio, liquid_level=liquid_level)
        except KeyboardInterrupt as e:
            break


def ball_pouring_control():
    # change to slight mode when ball full half of the cup
    # robot setting
    pouring_control = robot_setting()

    # camera setting
    cam = camera_setting()

    def read_img():
        image = xiapi.Image()
        cam.get_image(image)
        image_np = image.get_image_data_numpy()
        return True, image_np

    cam_roi = vf.RoiByFourPoints(read_img)

    n = 10
    while True:
        try:
            ratio = 0
            for i in range(n):
                ratio += cam_roi.get_foam_ratio(debug=True)
            ratio /= n
            liquid_level = pouring_control.max_height * cam_roi.get_liquid_level()
            pouring_control.ball_control(liquid_level)
            # if pouring_control.mode == 'slight':
            #     break
        except KeyboardInterrupt as e:
            break


def parallel_ball_pouring_control():
    pouring_control = robot_setting()
    vm = VisualMonitor()
    thread.start_new_thread(vm.monitor_pouring, ())
    print('start monitoring')

    while True:
        try:
            ratio = vm.get_ratio()
            if isinstance(ratio, float):
                liquid_level = pouring_control.max_height * vm.get_liquid_level()
                pouring_control.ball_control(liquid_level)
        except KeyboardInterrupt as e:
            print('quit')
            break


def parallel_beer_pouring_control():
    pouring_control = robot_setting()
    vm = VisualMonitor()
    thread.start_new_thread(vm.monitor_pouring, ())

    ratio_list = []
    liquid_level_list = []

    while True:
        try:
            ratio = vm.get_ratio()
            if isinstance(ratio, float):
                liquid_level = pouring_control.max_height * vm.get_liquid_level()
                pouring_control.auto_switch_control(ratio, liquid_level=liquid_level)
                if ratio != 1:
                    ratio_list.append(ratio)
                    liquid_level_list.append(liquid_level)
        except KeyboardInterrupt as e:
            plt.plot(ratio_list)
            plt.title('beer ratio during pouring')
            plt.xlabel('frame')
            plt.figure()
            plt.plot(liquid_level_list)
            plt.title('liquid level during pouring')
            plt.xlabel('frame')
            plt.ylabel('cm')
            plt.show()
            break


def ikfast_test():
    pouring_control = robot_setting()
    vm = VisualMonitor()
    thread.start_new_thread(vm.test_tracking, ())
    print('start monitoring')

    while True:
        try:
            pouring_control.switch_control('slight')
            time.sleep(1)
            pouring_control.switch_control('violent')
        except KeyboardInterrupt:
            print('quit')
            break


def pid_ball_test():
    pouring_control = robot_setting()
    vm = VisualMonitor()
    thread.start_new_thread(vm.monitor_pouring, ())
    print('start monitoring')
    time_list = []

    while True:
        try:
            time_start = time.time()
            ratio = vm.get_ratio()
            time_list.append((time.time() - time_start)*1000)
            if isinstance(ratio, float):
                liquid_level = pouring_control.max_height * vm.get_liquid_level()
                pouring_control.ball_control_pid(liquid_level)
        except KeyboardInterrupt as e:
            plt.plot(time_list)
            plt.title('time consumed during processing')
            plt.xlabel('frame')
            plt.ylabel('ms')
            plt.show()
            plt.figure(12)
            plt.subplot(121)
            plt.plot(vm.cam_roi.points_positions[0].x)
            plt.plot(vm.cam_roi.points_positions[1].x)
            plt.plot(vm.cam_roi.points_positions[2].x)
            plt.plot(vm.cam_roi.points_positions[3].x)
            plt.legend(('0', '1', '2', '3'), loc=0)
            plt.title('x positions')
            plt.subplot(122)
            plt.plot(vm.cam_roi.points_positions[0].y)
            plt.plot(vm.cam_roi.points_positions[1].y)
            plt.plot(vm.cam_roi.points_positions[2].y)
            plt.plot(vm.cam_roi.points_positions[3].y)
            plt.plot('y positions')
            plt.legend(('0', '1', '2', '3'), loc=0)
            print('quit')
            break


def pid_beer_test():
    pouring_control = robot_setting()
    vm = VisualMonitor()
    thread.start_new_thread(vm.monitor_pouring, ())

    ratio_list = []
    liquid_level_list = []

    while True:
        try:
            updated, ratio = vm.get_ratio()
            if updated:
                print('controlling')
                liquid_level = pouring_control.max_height * vm.get_liquid_level()
                pouring_control.beer_control_pid(ratio, liquid_level)
                if ratio != 1:
                    ratio_list.append(ratio)
                    liquid_level_list.append(liquid_level)
        except KeyboardInterrupt as e:
            plt.plot(ratio_list)
            plt.title('beer ratio during pouring')
            plt.xlabel('frame')
            plt.figure()
            plt.plot(liquid_level_list)
            plt.title('liquid level during pouring')
            plt.xlabel('frame')
            plt.ylabel('cm')
            plt.figure()
            plt.plot(vm.time_list)
            plt.title('time consumed during processing')
            plt.xlabel('frame')
            plt.ylabel('ms')
            plt.show()
            break


if __name__ == '__main__':
    # ikfast_test()
    # parallel_ball_pouring_control()
    # parallel_beer_pouring_control()
    # pid_ball_test()
    pid_beer_test()
