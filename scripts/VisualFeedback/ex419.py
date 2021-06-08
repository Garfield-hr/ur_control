from ex416 import *


def multi_two_region_max_diff(img):
    # input is binary img, output is dividing line and data point
    interval = 10
    nums = int(img.shape[1] / interval)
    num_rows = img.shape[0]
    beer_foam_points = []
    foam_air_points = []
    for i in range(nums):
        col = img[:, i*interval]
        beer_foam = 0
        foam_air = 0
        max_diff = 0
        min_diff = 0

        for ind, val in enumerate(col):
            if ind == 0: continue
            diff = sum(col[ind:]) / (num_rows - ind) - sum(col[:ind]) / ind
            if diff > max_diff:
                max_diff = diff
                beer_foam = ind
            if diff < min_diff:
                min_diff = diff
                foam_air = ind

        beer_foam_points.append(beer_foam)
        foam_air_points.append(foam_air)

    beer_foam_line = int(sum(beer_foam_points) / len(beer_foam_points))
    foam_air_line = int(sum(foam_air_points) / len(foam_air_points))
    return [beer_foam_line, foam_air_line], [beer_foam_points, foam_air_points]


if __name__ == '__main__':
    vid_dir = '/home/hairui/Videos/experiments/'
    video_name = '317-9D.avi'
    video_capture = cv.VideoCapture(vid_dir + video_name)

    cam_roi = RoiByFourPoints(video_capture.read)
    ret, img_roi = cam_roi.get_roi_img()
    areas = []
    bin_thresh = 160

    while ret:
        img_roi_g = cv.cvtColor(img_roi, cv.COLOR_BGR2GRAY)
        # get the area of two regions
        _, img_roi_bin = cv.threshold(img_roi_g, bin_thresh, 255, cv.THRESH_BINARY)
        lines, points = multi_two_region_max_diff(img_roi_bin)
        cv.line(img_roi, (0, lines[0]), (img_roi.shape[1], lines[0]), (255, 0, 0), 1, 0)
        #cv.line(img_roi, (0, lines[1]), (img_roi.shape[1], lines[1]), (0, 255, 0), 1, 0)
        beer_foam_points, foam_air_points = points
        for i in range(len(beer_foam_points)):
            ind = i*20
            cv.circle(img_roi, (ind, beer_foam_points[i]), 3, (255, 0, 0), -1)
            #cv.circle(img_roi, (ind, foam_air_points[i]), 3, (0, 255, 0), -1)

        cv.imshow('img', img_roi)
        if cv.waitKey(1) == 27:
            break
        ret, img_roi = cam_roi.get_roi_img()