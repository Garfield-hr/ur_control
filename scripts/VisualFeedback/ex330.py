import cv2 as cv
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt


class RoiByFourPoints:
    def __init__(self, next_image):
        self.get_next_image = next_image
        self.points = []
        self.roi_size = 10

        # initialize 4 points by mouse click
        ret, self.img = self.get_next_image()
        self.height, self.width = self.img.shape[:2]
        cv.imshow('img', self.img)
        cv.setMouseCallback('img', self.mouse_event)
        self.initialize_four_points()
        cv.destroyWindow('img')

    def mouse_event(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            self.points.append([x, y])
            cv.circle(self.img, (x, y), 1, (255, 255, 255), thickness=-1)
            cv.putText(self.img, xy, (x, y), cv.FONT_HERSHEY_PLAIN,
                       1.0, (255, 255, 255), thickness=1)
            cv.imshow('img', self.img)

    def initialize_four_points(self):
        while len(self.points) < 4:
            print(f'select four points for tracking, {4 - len(self.points)} points left')
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
    video_name = '317-9D.avi'
    video_capture = cv.VideoCapture(vid_dir + video_name)

    cam_roi = RoiByFourPoints(video_capture.read)
    ret, img_n = cam_roi.get_roi_img()
    areas = []

    while ret:
        # img_n = cv.cvtColor(img_n, cv.COLOR_BGR2GRAY)
        areas.append(foam_seg(img_n))
        ret, img_n = cam_roi.get_roi_img()

    plt.plot(areas)
    plt.show()


def func2():
    pic_dir = '/home/hairui/Pictures/experiment/'
    vid_dir = '/home/hairui/Videos/experiments/'
    img_name = '329-1.jpeg'
    video_name = '317-8D.avi'
    img = cv.imread(pic_dir+img_name)
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    def read_img():
        return True, img

    cam_roi = RoiByFourPoints(read_img)
    ret, img_roi = cam_roi.get_roi_img()
    _, img_bin = cv.threshold(img_roi, 160, 255, cv.THRESH_BINARY)
    ret, labels, stats, centroids = cv.connectedComponentsWithStats(img_bin)



if __name__ == '__main__':
    func1()


