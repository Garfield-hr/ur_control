import cv2 as cv
import numpy as np


def hsv_trackbar():
    default_values = (100, 100, 100, 200, 200, 200)
    cv.namedWindow("hsv_trackbar")
    cv.createTrackbar('LH', 'hsv_trackbar', default_values[0], 255, nothing)
    cv.createTrackbar('LS', 'hsv_trackbar', default_values[1], 255, nothing)
    cv.createTrackbar('LV', 'hsv_trackbar', default_values[2], 255, nothing)
    cv.createTrackbar('HH', 'hsv_trackbar', default_values[3], 255, nothing)
    cv.createTrackbar('HS', 'hsv_trackbar', default_values[4], 255, nothing)
    cv.createTrackbar('HV', 'hsv_trackbar', default_values[5], 255, nothing)


def nothing(x):
    pass


def hsv_seg(img, para=(6, 62, 0, 26, 255, 255)):
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    blurred_img = cv.GaussianBlur(hsv_img, (7, 7), 0)
    selected_img = cv.inRange(blurred_img, para[:3], para[3:])
    return selected_img


if __name__ == '__main__':
    # 8 220 106 20 255 255
    # 133 26 0 180 69 77
    # v = 165
    beer_img = cv.imread('/home/hairui/Pictures/experiment/cup.jpeg')
    cv.imshow('original_img', beer_img)
    blurred_img = cv.GaussianBlur(beer_img, (7, 7), 0)
    beer_hsv = cv.cvtColor(blurred_img, cv.COLOR_BGR2HSV)
    cv.imshow('hsv_img', beer_hsv)
    cv.waitKey(0)

    lower_bound = np.array([100, 100, 100])
    higher_bound = np.array([200, 200, 200])
    hsv_trackbar()

    while True:
        lower_bound[0] = cv.getTrackbarPos('LH', 'hsv_trackbar')
        lower_bound[1] = cv.getTrackbarPos('LS', 'hsv_trackbar')
        lower_bound[2] = cv.getTrackbarPos('LV', 'hsv_trackbar')
        higher_bound[0] = cv.getTrackbarPos('HH', 'hsv_trackbar')
        higher_bound[1] = cv.getTrackbarPos('HS', 'hsv_trackbar')
        higher_bound[2] = cv.getTrackbarPos('HV', 'hsv_trackbar')

        beer_select = cv.inRange(beer_hsv, lower_bound, higher_bound)
        cv.imshow('selected', beer_select)
        if cv.waitKey(33) == 27:
            break

    cv.destroyAllWindows()