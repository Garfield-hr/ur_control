from ex330 import *


def two_region_max_diff(img, draw=False):
    # binarize
    img_g = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    bin_thresh = 160
    _, img_bin = cv.threshold(img_g, bin_thresh, 255, cv.THRESH_BINARY)

    img_bin_row = np.sum(img_bin, axis=1)
    beer_foam = 0
    foam_air = 0
    max_diff = 0
    min_diff = 0
    num_rows = len(img_bin_row)

    for ind, val in enumerate(img_bin_row):
        if ind == 0: continue
        diff = sum(img_bin_row[ind:]) / (num_rows - ind) - sum(img_bin_row[:ind]) / ind
        if diff > max_diff:
            max_diff = diff
            beer_foam = ind
        if diff < min_diff:
            min_diff = diff
            foam_air = ind

    if draw:
        cv.line(img, (0, beer_foam), (img.shape[1], beer_foam), (0, 0, 255), 1, 0)
        cv.line(img, (0, foam_air), (img.shape[1], foam_air), (0, 255, 0), 1, 0)

    return beer_foam, foam_air


if __name__ == '__main__':
    vid_dir = '/home/hairui/Videos/experiments/'
    video_name = '317-9D.avi'
    video_capture = cv.VideoCapture(vid_dir + video_name)

    cam_roi = RoiByFourPoints(video_capture.read)
    ret, img_roi = cam_roi.get_roi_img()
    areas = []

    while ret:
        # img_roi_g = cv.cvtColor(img_roi, cv.COLOR_BGR2GRAY)
        # get the area of two regions
        two_region_max_diff(img_roi, draw=True)
        cv.imshow('img', img_roi)
        if cv.waitKey(1) == 27:
            break
        ret, img_roi = cam_roi.get_roi_img()

