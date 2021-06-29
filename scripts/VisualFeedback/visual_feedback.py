import math
import time
import thread
import cv2 as cv
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt


class ImgParaDis:
    def __init__(self, video_dir='/home/hairui/Videos/experiments/', video_name='new.avi', img_size=(1264, 1016)):
        fourcc = cv.VideoWriter_fourcc('I', '4', '2', '0')
        self.output_video = cv.VideoWriter(video_dir+video_name, fourcc, 24, img_size)
        self.img_dis = None
        self.img_save = None

    def update_img(self, img):
        self.img_dis = img

    def update_img_save(self, img):
        self.img_save = img

    def display_img(self):
        while True:
            if self.img_dis is not None:
                img_show = self.img_dis
                self.img_dis = None
                cv.imshow('parallel image display', img_show)
                cv.waitKey(33)

    def save_img(self):
        while True:
            if self.img_save is not None:
                img_save = self.img_save
                self.img_save = None
                self.output_video.write(img_save)

class PointPositions:
    def __init__(self):
        self.x = []
        self.y = []


class RoiByFourPoints:
    def __init__(self, next_image, parallel_display_img=None, parallel_save_img=None):
        self.get_next_image = next_image
        self.points = []
        self.roi_size = 10
        self.started = False
        self.liquid_level = 0.0
        self.upper_line = []
        self.lower_line = []
        self.inter_para = [0, 0, 0, 0, 0]
        self.filter_size = 9
        self.parallel_save_img = parallel_save_img
        self.points_positions = []
        for i in range(4):
            self.points_positions.append(PointPositions())

        def default_display_img(img):
            cv.imshow('image', img)
            cv.waitKey(1)

        if parallel_display_img is not None:
            self.parallel_display_img = parallel_display_img
        else:
            self.parallel_display_img = default_display_img

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

    def get_roi_img(self, extra_height=0):
        ret, img = self.get_next_image()
        if self.parallel_save_img is not None:
            self.parallel_save_img(img)
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

            for i in range(4):
                self.points_positions[i].x.append(self.points[i][0])
                self.points_positions[i].y.append(self.points[i][1])

            x1, x2 = min(self.points[0][0], self.points[1][0], self.points[2][0], self.points[3][0]), \
                     max(self.points[0][0], self.points[1][0], self.points[2][0], self.points[3][0])
            y1, y2 = min(self.points[0][1], self.points[1][1], self.points[2][1], self.points[3][1]), \
                     max(self.points[0][1], self.points[1][1], self.points[2][1], self.points[3][1])
            return ret, img[y1 - extra_height:y2, x1:x2].copy()
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

    def get_foam_ratio(self, debug=False, extra_height=0):
        ret, img = self.get_roi_img(extra_height=extra_height)
        if not ret:
            return
        if len(img.shape) == 3:
            img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        else:
            img_gray = img
        bin_thresh1 = 100
        bin_thresh2 = 120
        if not self.started:
            img_part = img_gray[img.shape[0]/2:]
            _, img_bin = cv.threshold(img_part, bin_thresh1, 255, cv.THRESH_BINARY)
            self.started = self.if_pouring_started(img_bin)
            if debug:
                self.parallel_display_img(img_bin)
            return 1.0
        else:
            _, img_bin = cv.threshold(img_gray, bin_thresh2, 255, cv.THRESH_BINARY)

        upper_line, lower_line = img_two_lines(img_bin)
        upper_line -= extra_height
        lower_line -= extra_height
        put_queue(upper_line, self.upper_line, self.filter_size)
        put_queue(lower_line, self.lower_line, self.filter_size)
        if len(self.upper_line) < self.filter_size:
            return 1.0
        upper_line = get_med_queue(self.upper_line)
        lower_line = get_med_queue(self.lower_line)

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
        self.inter_para = [h1, h2, h3, upper_line, lower_line]

        # for debug
        if debug:

            [x_lower_left_int, x_lower_right_int, x_upper_left_int, lower_line_int, upper_line_int] = map(int,
                                                                          [x_lower_left, x_lower_right, x_upper_left,
                                                                           lower_line + extra_height,upper_line + extra_height])
            cv.line(img, (0, lower_line_int), (img.shape[1], lower_line_int),
                    (0, 0, 255), 1, 0)
            cv.line(img, (0, upper_line_int), (img.shape[1], upper_line_int),
                    (0, 255, 0), 1, 0)
            cv.circle(img, (x_lower_left_int, lower_line_int), 3, (255, 0, 0), thickness=-1)
            cv.circle(img, (x_lower_right_int, lower_line_int), 3, (255, 0, 0), thickness=-1)
            cv.circle(img, (x_upper_left_int, upper_line_int), 3, (255, 0, 0), thickness=-1)
            cv.putText(img, 'h2=%f' % h2, (x_lower_left_int, lower_line_int), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 0, 0), thickness=1)
            cv.putText(img, 'h3=%f' % h3, (x_lower_right_int, lower_line_int), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 0, 0), thickness=1)
            cv.putText(img, 'h1=%f' % h1, (x_upper_left_int, upper_line_int), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 0, 0), thickness=1)
            ratio = "%f" % ((h2 + h3) / (2 * h1 + h2 + h3))
            cv.putText(img, ratio, (0, 10), cv.FONT_HERSHEY_PLAIN,
                       1.0, (0, 255, 0), thickness=1)
            length = "%f" % self.liquid_level
            cv.putText(img, length, (0, 50), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 255, 0), thickness=1)
            self.parallel_display_img(img)

        return (h2 + h3) / (2 * h1 + h2 + h3)

    def if_pouring_started(self, img_bin):
        min_length = 0.6 * (self.points[2][0] - self.points[1][0])
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

    def get_cal_h(self):
        return self.inter_para


def img_two_lines(img):
    if len(img.shape) == 3:
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # sobely = cv.Sobel(img, cv.CV_64F, 0, 1, ksize=5)
    # sobely_row = np.sum(sobely, axis=1)
    # upper_line = np.where(sobely_row == np.max(sobely_row))
    # lower_line = np.where(sobely_row == np.min(sobely_row))
    img_row = np.sum(img, axis=1)
    thresh = 0.5*max(img_row)
    img_row[img_row < thresh] = 0
    img_diff = np.diff(img_row.astype(np.int16))
    upper_line = np.where(img_diff == np.max(img_diff))
    lower_line = np.where(img_diff == np.min(img_diff))
    while not isinstance(upper_line, int):
        upper_line = upper_line[0]
    while not isinstance(lower_line, int):
        lower_line = lower_line[0]
    return upper_line, lower_line


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


def median_ave(data, n=9):
    ret = [np.median(data[max(0, ind - n/2): min(len(data), ind + n/2)]) for ind in range(len(data))]
    return ret


def put_queue(item, queue, max_size=9):
    if len(queue) < max_size:
        queue.append(item)
    elif len(queue) == max_size:
        queue.pop(0)
        queue.append(item)
    else:
        raise IndexError('queue is longer than max size')


def get_med_queue(queue):
    return np.median(queue)


def func1():
    pic_dir = '/home/hairui/Pictures/experiment/'
    vid_dir = '/home/hairui/Videos/experiments/'
    img_name = 'cut.jpeg'
    video_name = '618-1.avi'
    video_capture = cv.VideoCapture(vid_dir + video_name)
    parallel_displayer = ImgParaDis()
    thread.start_new_thread(parallel_displayer.display_img, ())
    # thread.start_new_thread(parallel_displayer.save_img, ())

    cam_roi = RoiByFourPoints(video_capture.read, parallel_displayer.update_img, parallel_displayer.update_img_save)
    # cam_roi = RoiByFourPoints(video_capture.read)
    time_consumed = []
    time_start = time.time()
    ratio = cam_roi.get_foam_ratio(debug=True)
    time_consumed.append((time.time() - time_start) * 1000)
    ratio_list = []
    liquid_level_arr = []
    h1 = []
    h2 = []
    h3 = []
    ul = []
    ll = []
    while isinstance(ratio, (int, float)):

        time_start = time.time()
        ratio = cam_roi.get_foam_ratio(debug=True, extra_height=30)
        heights = cam_roi.get_cal_h()
        if ratio != 1:
            ratio_list.append(ratio)
            liquid_level_arr.append(cam_roi.get_liquid_level() * 11)
            h1.append(heights[0])
            h2.append(heights[1])
            h3.append(heights[2])
            ul.append(heights[3])
            ll.append(heights[4])
        time_consumed.append((time.time() - time_start) * 1000)

    plt.plot(ratio_list)
    plt.figure()
    plt.plot(time_consumed)
    plt.figure(23)
    plt.subplot(231)
    plt.plot(liquid_level_arr)
    plt.subplot(232)
    plt.plot(h1)
    plt.subplot(233)
    plt.plot(h2)
    plt.subplot(234)
    plt.plot(h3)
    plt.subplot(235)
    plt.plot(ul)
    plt.subplot(236)
    plt.plot(ll)
    plt.figure(12)
    plt.subplot(121)
    plt.plot(cam_roi.points_positions[0].x)
    plt.plot(cam_roi.points_positions[1].x)
    plt.plot(cam_roi.points_positions[2].x)
    plt.plot(cam_roi.points_positions[3].x)
    plt.legend(('0', '1', '2', '3'),  loc=0)
    plt.title('x positions')
    plt.subplot(122)
    plt.plot(cam_roi.points_positions[0].y)
    plt.plot(cam_roi.points_positions[1].y)
    plt.plot(cam_roi.points_positions[2].y)
    plt.plot(cam_roi.points_positions[3].y)
    plt.plot('y positions')
    plt.legend(('0', '1', '2', '3'), loc=0)
    plt.show()
    print(sum(time_consumed) / len(time_consumed))
    parallel_displayer.output_video.release()


def func2():
    import time
    pic_dir = '/home/hairui/Pictures/experiment/'
    vid_dir = '/home/hairui/Videos/experiments/'
    img_name = 'bad1819.jpeg'
    video_name = '616-2.avi'
    img = cv.imread(pic_dir + img_name)
    ret = True
    def read_img():
        return ret, img

    parallel_displayer = ImgParaDis()
    thread.start_new_thread(parallel_displayer.display_img, ())

    cam_roi = RoiByFourPoints(read_img)
    # cam_roi = RoiByFourPoints(video_capture.read)
    _, img_roi = cam_roi.get_roi_img()
    img_gray = cv.cvtColor(img_roi, cv.COLOR_BGR2GRAY)
    bin_thresh = 100
    _, img_bin = cv.threshold(img_gray, bin_thresh, 255, cv.THRESH_BINARY)
    # sobely = cv.Sobel(img, cv.CV_64F, 0, 1, ksize=5)
    # sobely_row = np.sum(sobely, axis=1)
    # upper_line = np.where(sobely_row == np.max(sobely_row))
    # lower_line = np.where(sobely_row == np.min(sobely_row))
    # while not isinstance(upper_line, int):
    #     upper_line = upper_line[0]
    # while not isinstance(lower_line, int):
    #     lower_line = lower_line[0]
    img_bin_row = np.sum(img_bin, axis=1)
    max_val = 0.5*max(img_bin_row)
    img_bin_row[img_bin_row < max_val] = 0
    # img_bin_row[img_bin_row > max_val] = 1
    img_bin_row_type = img_bin_row.astype(np.int16)
    img_bin_row_diff = np.diff(img_bin_row_type)
    upper_line = np.where(img_bin_row_diff == np.max(img_bin_row_diff))[0]
    lower_line = np.where(img_bin_row_diff == np.min(img_bin_row_diff))[0]
    cv.line(img_roi, (0, lower_line), (img_roi.shape[1], lower_line), (0, 0, 255), 1, 0)
    cv.line(img_roi, (0, upper_line), (img_roi.shape[1], upper_line), (0, 255, 0), 1, 0)
    print(upper_line, lower_line)

    cv.imshow('img_bin', img_bin)
    cv.imshow('img', img_roi)
    cv.waitKey(0)
    plt.plot(img_bin_row)
    plt.figure()
    plt.plot(img_bin_row_diff)
    plt.show()



if __name__ == '__main__':
    func1()
