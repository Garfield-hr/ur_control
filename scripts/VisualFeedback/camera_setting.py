from ximea import xiapi
import cv2


def adjust_camera(cam):
    img =xiapi.Image()
    while cv2.waitKey(33) != 27:
        cam.get_image(img)
        data = img.get_image_data_numpy()
        cv2.imshow("img", data)


def white_balance_adjustment(cam):
    print('please adjust the camera for white balance, esc to continue')
    adjust_camera(cam)
    cam.set_param(xiapi.XI_PRM_MANUAL_WB, 1)
    print('is the image corrected? esc to continue')
    adjust_camera(cam)


cam = xiapi.Camera()
# start communication
print('Opening first camera...')
cam.open_device()

# settings
cam.set_imgdataformat('XI_RGB24')
cam.set_exposure(792)
cam.set_region_selector(0)
cam.set_width(640)
cam.set_height(600)
cam.set_gain(15)
img = xiapi.Image()

# start data acquisition
print('Starting data acquisition...')
cam.start_acquisition()

