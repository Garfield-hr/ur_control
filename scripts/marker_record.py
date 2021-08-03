import numpy as np
import time

import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import cv2
import pandas as pd
from pouring_control import robot_setting

def callback1(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard")

if __name__ == '__main__':
    pouring_control = robot_setting()
    ratio_list = []
    liquid_level_list = []
    time_list = []
    time_abs_list = []
    time_read_list = []
    time_proc_list = []
    marker_position_0_x = []
    marker_position_0_y = []
    marker_position_1_x = []
    marker_position_1_y = []
    marker_position_2_x = []
    marker_position_2_y = []
    marker_position_3_x = []
    marker_position_3_y = []

    h_list = []
    ratio_sum = 0.0
    ratio_num = 0.0
    h = 0
    def callback(data):
        global h
        ratio_list.append(data.data[0])
        liquid_level_list.append(data.data[1])
        time_abs_list.append(data.data[5] / 1000)
        h_list.append(h)
        marker_position_0_x.append(data.data[6])
        marker_position_0_y.append(data.data[7])
        marker_position_1_x.append(data.data[8])
        marker_position_1_y.append(data.data[9])
        marker_position_2_x.append(data.data[10])
        marker_position_2_y.append(data.data[11])
        marker_position_3_x.append(data.data[12])
        marker_position_3_y.append(data.data[13])
        time_list.append(data.data[2])
        time_read_list.append(data.data[3])
        time_proc_list.append(data.data[4])


    rospy.init_node("pouring_control", anonymous=True)
    rospy.Subscriber("BeerPouring", Float64MultiArray, callback)
    print("running")
    # cv2.namedWindow("waiting",cv2.WINDOW_NORMAL)
    # cv2.waitKey(0)
    for i in range(10):
        pouring_control.switch_control('violent')
        pouring_control.switch_control('slight')
        pouring_control.switch_control('high')
        time.sleep(1)
    print('plotting')
    t_start = time_abs_list[0]
    map(lambda x: x - t_start, time_abs_list)
    # plt.plot(time_abs_list, ratio_list)
    # plt.title('beer ratio during pouring')
    # plt.xlabel('s')
    # plt.figure()
    # plt.plot(time_abs_list, h_list)
    # plt.title('cup height change during pouring')
    # plt.xlabel('s')
    # plt.ylabel('m')
    # plt.figure()
    # plt.plot(time_abs_list, liquid_level_list)
    # plt.title('liquid level during pouring')
    # plt.xlabel('s')
    # plt.ylabel('cm')
    # plt.figure()
    # plt.plot(time_list)
    # plt.title('time consumed during pouring')
    # plt.xlabel('frame')
    # plt.ylabel('ms')
    # plt.figure()
    # plt.plot(time_read_list)
    # plt.title('time consumed during reading')
    # plt.xlabel('frame')
    # plt.ylabel('ms')
    # plt.figure()
    # plt.plot(time_proc_list)
    # plt.title('time consumed during processing')
    # plt.xlabel('frame')
    # plt.ylabel('ms')
    plt.show()
    data_save = []
    data_save.append(time_abs_list)
    data_save.append(marker_position_0_x)
    data_save.append(marker_position_0_y)
    data_save.append(marker_position_1_x)
    data_save.append(marker_position_1_y)
    data_save.append(marker_position_2_x)
    data_save.append(marker_position_2_y)
    data_save.append(marker_position_3_x)
    data_save.append(marker_position_3_y)
    data_save = np.array(data_save)
    data_save = data_save.T
    save = pd.DataFrame(data_save, columns=['time', 'x0', 'y0', 'x1', 'y1', 'x2', 'y2', 'x3', 'y3'])
    save.plot()
    plt.show()
    doc_dir = '/home/liangxiao/Documents/experiment/'
    data_name = raw_input('input the name of data, including .csv')
    save.to_csv(doc_dir+data_name, index=False, header=False)

