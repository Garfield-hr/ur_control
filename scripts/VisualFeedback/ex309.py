import cv2 as cv
import numpy as np
from hsvSeg import hsv_seg


def img_find_line(img, draw=True):
    selected_img = hsv_seg(img)
    selected_img_row = np.sum(selected_img, axis=1)
    best_seg = 0
    max_diff = 0
    num_rows = len(selected_img_row)
    for ind, val in enumerate(selected_img_row):
        if ind == 0: continue
        diff = sum(selected_img_row[ind:])/(num_rows - ind) - sum(selected_img_row[:ind])/ind
        if diff > max_diff:
            max_diff = diff
            best_seg = ind

    if draw:
        cv.line(img, (0, best_seg), (img.shape[1], best_seg), (0, 0, 255), 1, 8)
    return best_seg


if __name__ == '__main__':
    video_dir_name = '/home/hairui/Videos/experiments/'

    video_name = '317-1.avi'
    video_capture = cv.VideoCapture(video_dir_name+video_name)
    if video_capture.isOpened():
        ret = True
        while ret:
            ret, img = video_capture.read()
            #img = cv.flip(img, 0)  # images in this video is upsidedown
            img_find_line(img)
            cv.imshow('video', img)
            if cv.waitKey(5) == 27: break

