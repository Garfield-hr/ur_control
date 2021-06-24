import cv2 as cv


def jump_frame(frame):
    global vc
    vc.set(cv.CAP_PROP_POS_FRAMES, frame)
    print(frame)


def cut_image():
    vc = cv.VideoCapture('/home/hairui/Videos/experiments/616-2.avi')
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


def cut_video():
    vc = cv.VideoCapture('/home/hairui/Videos/experiments/616-1.avi')
    frame_num = int(vc.get(cv.CAP_PROP_FRAME_COUNT))
    cv.namedWindow('video', cv.WINDOW_NORMAL)
    cv.createTrackbar('frame', 'video', 0, frame_num, jump_frame)

    fourcc = cv.VideoWriter_fourcc('I', '4', '2', '0')
    output_video = cv.VideoWriter("/home/hairui/Videos/experiments/616-2.avi", fourcc, 24, (1264, 1016))
    if vc.isOpened():
        ret = True
        while ret:
            ret, img = vc.read()
            if ret:
                cv.setTrackbarPos('frame', 'video', int(vc.get(cv.CAP_PROP_POS_FRAMES)))
                cv.imshow('video', img)
                if cv.waitKey(33) == 27:
                    ret1, img = vc.read()
                    while ret1:
                        output_video.write(img)
                        ret1, img = vc.read()
                    break


if __name__ == '__main__':
    vc = cv.VideoCapture('/home/hairui/Videos/experiments/618-1.avi')
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
                    str_name = raw_input('Please input the name of this image')
                    cv.imwrite('/home/hairui/Pictures/experiment/' + str_name, img)
                    break