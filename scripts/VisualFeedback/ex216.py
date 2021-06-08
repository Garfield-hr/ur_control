import cv2 as cv
import numpy as np
from imgSeg import dir_str, img_str
from hsvSeg import hsv_seg


def hsv_dialation():
    img = cv.imread(dir_str + img_str)
    img = cv.flip(img, 0)
    cv.imshow('img', img)
    selected_img = hsv_seg(img)

    # kernel = np.ones((5, 5), np.uint8)
    # dialation = cv.dilate(selected_img, kernel)
    # cv.imshow('final', dialation)
    cv.imshow('final', selected_img)
    cv.waitKey(0)


def edge_detection():
    img = cv.imread(dir_str + img_str)
    img = cv.flip(img, 0)
    cv.imshow('img', img)
    # hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    blurred_img = cv.GaussianBlur(img, (7, 7), 0)
    cv.imshow('blurred', blurred_img)
    canny = cv.Canny(blurred_img, 10, 40)
    cv.imshow('edge', canny)
    cv.waitKey(0)
    img_name = input('please input the name of the image to be saved')
    cv.imwrite(dir_str+img_name+'_original.jpeg', img)
    cv.imwrite(dir_str + img_name + '_blur.jpeg',blurred_img)
    cv.imwrite(dir_str+img_name+'_edge.jpeg', canny)


if __name__ == '__main__':
    # vc = cv.VideoCapture('/home/hairui/Videos/experiments/317-1.avi')
    # if vc.isOpened():
    #     ret = True
    #     while ret:
    #         ret, frame = vc.read()
    #         img = cv.flip(frame, 0)
    #         selected = hsv_seg(img)
    #         cv.imshow('original', img)
    #         cv.imshow('selected', selected)
    #         if cv.waitKey(33) == 27:
    #             break
    #
    # cv.destroyAllWindows()
    edge_detection()
