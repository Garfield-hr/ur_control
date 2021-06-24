from ximea import xiapi
import cv2
import time
from camera_setting import white_balance_adjustment

def adjust_camera(cam):
    img =xiapi.Image()
    while cv2.waitKey(33) != 27:
        cam.get_image(img)
        data = img.get_image_data_numpy()
        cv2.imshow("img", data)


if __name__ == '__main__':
    cam = xiapi.Camera()
    # start communication
    print('Opening first camera...')
    cam.open_device()

    # settings
    cam.set_imgdataformat('XI_RGB24')
    cam.set_exposure(1048)
    # cam.set_region_selector(0)
    cam.set_width(1264)
    cam.set_height(1016)
    cam.set_gain(5)

    # create instance of Image to store image data and metadata
    img = xiapi.Image()

    # start data acquisition
    print('Starting data acquisition...')
    cam.start_acquisition()

    white_balance_adjustment(cam)

    adjust_camera(cam)
    a = raw_input('press any key to start recording, esc to stop')

    fourcc = cv2.VideoWriter_fourcc('I', '4', '2', '0')
    output_video = cv2.VideoWriter("/home/hairui/Videos/experiments/618-1.avi", fourcc, 24, (1264, 1016))

    try:
        while cv2.waitKey(1) != 27:
            cam.get_image(img)
            data = img.get_image_data_numpy()

            output_video.write(data)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()