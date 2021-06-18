import matplotlib.pyplot as plt
import VisualFeedback.visual_feedback as vf
from VisualFeedback.visual_feedback import ImgParaDis
from ximea import xiapi
import cv2 as cv
from RobotControl.pouring_feedback import FeedbackPouringControl, ControlMode, MyRobotPlanner, UrControl
from VisualFeedback.camera_setting import white_balance_adjustment
import rospy
import thread
import time


class VisualMonitor:
    __ratio_list = []
    __liquid_level = 0.0
    __time_consumed = []

    def __init__(self, parallel_display=None):
        cam = camera_setting()

        def read_img():
            image = xiapi.Image()
            cam.get_image(image)
            image_np = image.get_image_data_numpy()
            return True, image_np

        self.cam_roi = vf.RoiByFourPoints(read_img, parallel_display)
        self.__stop = False
        self.parallel_dis = parallel_display

    def monitor_pouring(self):
        while not self.__stop:
            time_start = time.time()
            self.__ratio_list.append(self.cam_roi.get_foam_ratio(debug=True))
            self.__liquid_level = self.cam_roi.get_liquid_level()
            self.__time_consumed.append(1000*(time.time() - time_start))

    def get_ratio(self):
        if len(self.__ratio_list) > 10:
            return 0.1*sum(self.__ratio_list[-10:])

    def stop_monitoring(self):
        self.__stop = True

    def get_liquid_level(self):
        return self.__liquid_level

    def test_tracking(self):
        while not self.__stop:
            _, img = self.cam_roi.get_roi_img()
            cv.imshow('img', img)
            cv.waitKey(1)

    def plot_time(self):
        plt.plot(self.__time_consumed)
        plt.show()

    def camera_test(self):
        _, img = self.cam_roi.get_roi_img()
        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        _, img_bin = cv.threshold(img_gray, 120, 255, cv.THRESH_BINARY)
        self.parallel_dis(img_bin)


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
    pouring_control = FeedbackPouringControl(robot, [0.11, 0.03])
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

    parallel_displayer = ImgParaDis()
    thread.start_new_thread(parallel_displayer.display_img, ())

    cam_roi = vf.RoiByFourPoints(read_img, parallel_displayer.update_img)

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
    parallel_displayer = ImgParaDis()
    thread.start_new_thread(parallel_displayer.display_img, ())

    vm = VisualMonitor(parallel_displayer.update_img)
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
    parallel_displayer = ImgParaDis()
    thread.start_new_thread(parallel_displayer.display_img, ())

    vm = VisualMonitor(parallel_displayer.update_img)
    thread.start_new_thread(vm.monitor_pouring, ())


    while True:
        try:
            ratio = vm.get_ratio()
            if isinstance(ratio, float):
                liquid_level = pouring_control.max_height * vm.get_liquid_level()
                print('ratio is', ratio, 'level is', liquid_level)
                pouring_control.auto_switch_control(ratio, liquid_level=liquid_level)
        except KeyboardInterrupt as e:
            print('quit')
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

def camera_test():
    parallel_displayer = ImgParaDis()
    thread.start_new_thread(parallel_displayer.display_img, ())

    vm = VisualMonitor(parallel_displayer.update_img)
    # thread.start_new_thread(vm.monitor_pouring, ())
    while True:
        try:
            vm.camera_test()
        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    # ikfast_test()
    # parallel_ball_pouring_control()
    parallel_beer_pouring_control()
    # camera_test()
