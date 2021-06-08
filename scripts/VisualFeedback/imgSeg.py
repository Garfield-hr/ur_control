import cv2 as cv
import numpy as np
from copy import deepcopy


dir_str = '/home/hairui/Pictures/experiment/'
img_str = 'test_image.png'


class MouseInput:
    def __init__(self, img):
        self.click_count = 0
        self.left_up = (0, 0)
        self.right_down = (0, 0)
        self.img = deepcopy(img)
        cv.imshow('image', self.img)
        cv.setMouseCallback('image', self.mouse_event)

    def mouse_event(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            if self.click_count > 1:
                print('input more than 2 points, clean all data, please retry')
                self.click_count = 0
            elif self.click_count == 0:
                self.left_up = (x, y)
                self.click_count += 1
                cv.circle(self.img, (x, y), 1, (255, 255, 255), thickness=-1)
                cv.putText(self.img, xy, (x, y), cv.FONT_HERSHEY_PLAIN,
                            1.0, (255, 255, 255), thickness=1)
                cv.imshow('image', self.img)
            else:
                self.right_down = (x, y)
                self.click_count += 1
                cv.circle(self.img, (x, y), 1, (255, 255, 255), thickness=-1)
                cv.putText(self.img, xy, (x, y), cv.FONT_HERSHEY_PLAIN,
                           1.0, (255, 255, 255), thickness=1)
                cv.imshow('image', self.img)

    def get_rect(self):
        if self.click_count == 0:
            print('please select two points')
        while self.click_count != 2 or (self.left_up[0] >= self.right_down[0]) or (self.left_up[1] >= self.right_down[1]):
            if self.click_count == 2:
                print('x1,y1', self.left_up,'x2,y2',self.right_down)
                print('invalid input, try again')
                self.click_count = 0
            cv.waitKey(100)
        return self.left_up[0], self.left_up[1], self.right_down[0] - self.left_up[0], self.right_down[1] - self.left_up[1]


def grab_cut(img):
    mask = np.zeros(img.shape[:2], np.uint8)
    bgdModel = np.zeros((1, 65), np.float64)
    fgdModel = np.zeros((1, 65), np.float64)
    window_name = 'image'
    cv.namedWindow(window_name)
    cv.imshow(window_name, img)
    MouseIn = MouseInput(img)
    rect = MouseIn.get_rect()
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    cv.grabCut(img_hsv, mask, rect, bgdModel, fgdModel, 5, cv.GC_INIT_WITH_RECT)

    mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
    img = img*mask2[:, :, np.newaxis]
    cv.imshow('dst', img)
    cv.waitKey(0)
    cv.imwrite(dir_str+'output.jpeg', img)


if __name__ == '__main__':
    img = cv.imread(dir_str + img_str)
    img = cv.flip(img, 0)
    if not img.size:
        print('can not open %s' % img_str)
        exit(1)
    grab_cut(img)
    cv.destroyAllWindows()


