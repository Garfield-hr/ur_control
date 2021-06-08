from ximea import xiapi
import cv2
from camera_setting import white_balance_adjustment
from main import adjust_camera

cam = xiapi.Camera()
# start communication
print('Opening first camera...')
cam.open_device()

# settings
cam.set_imgdataformat('XI_RGB24')
# cam.set_exposure(792)
# cam.set_region_selector(0)
# cam.set_width(1264)
# cam.set_height(1016)
# cam.set_gain(15)

# create instance of Image to store image data and metadata
img = xiapi.Image()

# start data acquisition
print('Starting data acquisition...')
#cam.start_acquisition()

white_balance_adjustment(cam)
# print('please adjust camera for pouring measurement')
# adjust_camera(cam)

try:
    while cv2.waitKey(1) != 27:
        cam.get_image(img)
        data = img.get_image_data_numpy()
        img_gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, 160, 255, cv2.THRESH_BINARY)

        cv2.imshow('original', data)
        cv2.imshow('binary', img_bin)

except KeyboardInterrupt:
    cv2.destroyAllWindows()