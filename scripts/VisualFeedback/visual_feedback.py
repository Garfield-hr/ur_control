import math

import cv2 as cv
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt


class RoiByFourPoints:
    def __init__(self, next_image):
        self.get_next_image = next_image
        self.points = []
        self.roi_size = 10
        self.started = False
        self.liquid_level = 0.0

        # initialize 4 points by mouse click
        ret, self.img = self.get_next_image()
        self.height, self.width = self.img.shape[:2]
        cv.imshow('image', self.img)
        cv.setMouseCallback('image', self.mouse_event)
        self.initialize_four_points()
        cv.destroyWindow('image')

    def mouse_event(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            self.points.append([x, y])
            cv.circle(self.img, (x, y), 1, (255, 255, 255), thickness=-1)
            cv.putText(self.img, xy, (x, y), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 255, 255), thickness=1)
            cv.imshow('image', self.img)

    def initialize_four_points(self):
        while len(self.points) < 4:
            print('select four points for tracking, %d points left' % (4 - len(self.points)))
            cv.waitKey(1000)

    def get_roi_img(self):
        ret, img = self.get_next_image()
        if ret:
            for ind, point in enumerate(self.points):

                x, y = point
                img_roi = deepcopy(img[max(0, y - self.roi_size):min(self.height, y + self.roi_size),
                                   max(0, x - self.roi_size): min(self.width, x + self.roi_size)])
                # transfer to binary
                if len(img_roi.shape) == 3:
                    img_roi = cv.cvtColor(img_roi.copy(), cv.COLOR_BGR2GRAY)
                _, img_roi_bin = cv.threshold(img_roi, 220, 255, cv.THRESH_BINARY)
                mom = cv.moments(img_roi_bin)
                if mom['m00'] > 20:
                    x = int(mom['m10'] / mom['m00']) + x - self.roi_size
                    y = int(mom['m01'] / mom['m00']) + y - self.roi_size
                self.points[ind] = [x, y]

            x1, x2 = min(self.points[0][0], self.points[1][0], self.points[2][0], self.points[3][0]), \
                     max(self.points[0][0], self.points[1][0], self.points[2][0], self.points[3][0])
            y1, y2 = min(self.points[0][1], self.points[1][1], self.points[2][1], self.points[3][1]), \
                     max(self.points[0][1], self.points[1][1], self.points[2][1], self.points[3][1])
            return ret, img[y1:y2, x1:x2].copy()
        return ret, ret

    def track_four_points(self):
        ret, img = self.get_next_image()
        while ret:
            for ind, point in enumerate(self.points):
                x, y = point
                img_roi = img[max(0, y - self.roi_size):min(self.height, y + self.roi_size),
                          max(0, x - self.roi_size): min(self.width, x + self.roi_size)]
                # transfer to binary
                if len(img_roi.shape) == 3:
                    img_roi = cv.cvtColor(img_roi, cv.COLOR_BGR2GRAY)
                _, img_roi_bin = cv.threshold(img_roi, 220, 255, cv.THRESH_BINARY)
                mom = cv.moments(img_roi_bin)
                if mom['m00'] > 20:
                    x = int(mom['m10'] / mom['m00'])
                    y = int(mom['m01'] / mom['m00'])
                self.points[ind] = [x, y]

            x1, x2 = min(self.points[0][0], self.points[1][0], self.points[2][0], self.points[3][0]), \
                     max(self.points[0][0], self.points[1][0], self.points[2][0], self.points[3][0])
            y1, y2 = min(self.points[0][1], self.points[1][1], self.points[2][1], self.points[3][1]), \
                     max(self.points[0][1], self.points[1][1], self.points[2][1], self.points[3][1])
            self.img_roi = img[y1:y2, x1:x2]
            ret, img = self.get_next_image()

    def get_foam_ratio(self, debug=False):
        ret, img = self.get_roi_img()
        if not ret:
            return
        if len(img.shape) == 3:
            img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        bin_thresh = 160
        _, img_bin = cv.threshold(img_gray, bin_thresh, 255, cv.THRESH_BINARY)
        if not self.started:
            self.started = self.if_pouring_started(img_bin)
            return 0.0

        img_bin_row = np.sum(img_bin, axis=1)
        lower_line = 0
        upper_line = 0
        max_diff = 0
        min_diff = 0
        num_rows = len(img_bin_row)

        for ind, val in enumerate(img_bin_row):
            if ind == 0: continue
            diff = sum(img_bin_row[ind:]) / (num_rows - ind) - sum(img_bin_row[:ind]) / ind
            if diff > max_diff:
                max_diff = diff
                upper_line = ind
            if diff < min_diff:
                min_diff = diff
                lower_line = ind

        # oordinate of self.points and lines are not same
        # fix it and add function to show result in figures

        def x_by_two_points_and_y(xa, ya, xb, yb, y):
            return xb + (y - yb) * (xb - xa) / (yb - ya)

        x_lower_left = x_by_two_points_and_y(0, 0,
                                             self.points[1][0] - self.points[0][0],
                                             self.points[1][1] - self.points[0][1],
                                             lower_line)
        x_lower_right = x_by_two_points_and_y(self.points[2][0] - self.points[0][0],
                                              self.points[2][1] - self.points[0][1],
                                              self.points[3][0] - self.points[0][0],
                                              self.points[3][1] - self.points[0][1],
                                              lower_line)
        x_upper_left = x_by_two_points_and_y(0, 0,
                                             self.points[1][0] - self.points[0][0],
                                             self.points[1][1] - self.points[0][1],
                                             upper_line)
        # h1 upper line to lower line
        # h2 lower left to left corner
        # h3 lower right to right corner
        h1 = math.sqrt((x_upper_left - x_lower_left) ** 2 + (upper_line - lower_line) ** 2)
        h2 = math.sqrt((x_lower_left - (self.points[1][0] - self.points[0][0])) ** 2 +
                       (lower_line - (self.points[1][1] - self.points[0][1])) ** 2)
        h3 = math.sqrt((x_lower_right - (self.points[2][0] - self.points[0][0])) ** 2 +
                       (lower_line - (self.points[2][1] - self.points[0][1])) ** 2)

        # update liquid level
        self.liquid_level = (self.points[1][1] - self.points[0][1] - upper_line) \
                            / float(self.points[1][1] - self.points[0][1])

        # for debug
        if debug:
            cv.line(img, (0, lower_line), (img.shape[1], lower_line), (0, 0, 255), 1, 0)
            cv.line(img, (0, upper_line), (img.shape[1], upper_line), (0, 255, 0), 1, 0)
            [x_lower_left_int, x_lower_right_int, x_upper_left_int] = map(int,
                                                                          [x_lower_left, x_lower_right, x_upper_left])
            cv.circle(img, (x_lower_left_int, lower_line), 3, (255, 0, 0), thickness=-1)
            cv.circle(img, (x_lower_right_int, lower_line), 3, (255, 0, 0), thickness=-1)
            cv.circle(img, (x_upper_left_int, upper_line), 3, (255, 0, 0), thickness=-1)
            cv.putText(img, 'h2=%f' % h2, (x_lower_left_int, lower_line), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 0, 0), thickness=1)
            cv.putText(img, 'h3=%f' % h3, (x_lower_right_int, lower_line), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 0, 0), thickness=1)
            cv.putText(img, 'h1=%f' % h1, (x_upper_left_int, upper_line), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 0, 0), thickness=1)
            ratio = "%f" % ((h2 + h3) / (2 * h1 + h2 + h3))
            cv.putText(img, ratio, (10, 10), cv.FONT_HERSHEY_PLAIN,
                       1.0, (0, 255, 0), thickness=1)
            length = "%f" % self.liquid_level
            cv.putText(img, length, (0, 50), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 255, 0), thickness=1)
            cv.imshow('img', img)
            cv.waitKey(1)

        return (h2 + h3) / (2 * h1 + h2 + h3)

    def if_pouring_started(self, img_bin):
        min_length = 0.75 * (self.points[2][0] - self.points[1][0])
        img_selected = np.zeros((img_bin.shape[0], img_bin.shape[1], 3), dtype=np.uint8)
        ret, labels, stats, centroids = cv.connectedComponentsWithStats(img_bin)
        area = 0
        this_img_started = False
        for ind, stat in enumerate(stats):
            if not ind: continue
            if stat[2] > min_length:
                area += np.sum(labels == ind)
                img_selected[labels == ind, 2] = 255
                this_img_started = True
        return this_img_started

    def get_liquid_level(self):
        return self.liquid_level


def foam_seg(image):
    img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    threshold = 160
    min_width = 35
    _, img_bin = cv.threshold(img, threshold, 255, cv.THRESH_BINARY)
    img_bin[:10, :] = 0
    img_selected = np.zeros((img_bin.shape[0], img_bin.shape[1], 3), dtype=np.uint8)
    ret, labels, stats, centroids = cv.connectedComponentsWithStats(img_bin)
    area = 0
    for ind, stat in enumerate(stats):
        if not ind: continue
        if stat[2] > min_width:
            area += np.sum(labels == ind)
            img_selected[labels == ind, 2] = 255

    cv.imshow('original', image)
    cv.imshow('binary', img_bin)
    cv.imshow('selected', img_selected)
    cv.waitKey(1)
    return area


def func1():
    pic_dir = '/home/hairui/Pictures/experiment/'
    vid_dir = '/home/hairui/Videos/experiments/'
    img_name = '329-1.jpeg'
    video_name = 'ball.avi'
    video_capture = cv.VideoCapture(vid_dir + video_name)

    cam_roi = RoiByFourPoints(video_capture.read)
    ratio = cam_roi.get_foam_ratio(debug=True)
    ratio_list = []
    while isinstance(ratio, (int, float)):
        ratio_list.append(ratio)
        ratio = cam_roi.get_foam_ratio(debug=True)

    plt.plot(ratio_list)
    plt.show()


def func2():
    pic_dir = '/home/hairui/Pictures/experiment/'
    vid_dir = '/home/hairui/Videos/experiments/'
    img_name = '329-1.jpeg'
    video_name = '317-8D.avi'
    img = cv.imread(pic_dir + img_name)
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    def read_img():
        return True, img

    cam_roi = RoiByFourPoints(read_img)
    ret, img_roi = cam_roi.get_roi_img()
    _, img_bin = cv.threshold(img_roi, 160, 255, cv.THRESH_BINARY)
    ret, labels, stats, centroids = cv.connectedComponentsWithStats(img_bin)


if __name__ == '__main__':
    func1()
