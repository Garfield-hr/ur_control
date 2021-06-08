import cv2 as cv


def jump_frame(frame):
    global vc
    vc.set(cv.CAP_PROP_POS_FRAMES, frame)
    print(frame)


if __name__ == '__main__':
    vc = cv.VideoCapture('/home/hairui/Videos/experiments/317-9D.avi')
    frame_num = int(vc.get(cv.CAP_PROP_FRAME_COUNT))
    cv.namedWindow('video', cv.WINDOW_NORMAL)
    cv.createTrackbar('frame', 'video', 0, frame_num, jump_frame)
    if vc.isOpened():
        ret = True
        while ret:
            ret, img = vc.read()
            if ret:
                cv.setTrackbarPos('frame', 'video', int(vc.get(cv.CAP_PROP_POS_FRAMES)))
                cv.imshow('video', img)
                if cv.waitKey(33) == 27:
                    str_name = input('Please input the name of this image')
                    cv.imwrite('/home/hairui/Pictures/experiment/' + str_name, img)
                    break