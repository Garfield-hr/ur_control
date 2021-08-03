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
    h_list = []
    ratio_sum = 0.0
    ratio_num = 0.0
    h = 0
    def callback(data):
        global h
        if data.data[0] > 1e-3:
            global ratio_sum, ratio_num
            ratio_num += 1
            ratio_sum += data.data[0]
            if ratio_num == 10:
                h = pouring_control.beer_control_pid(ratio_sum/10, data.data[1])
                print("ratio is", ratio_sum / 10)
                ratio_sum = 0.0
                ratio_num = 0

            ratio_list.append(data.data[0])
            liquid_level_list.append(data.data[1])
            time_abs_list.append(data.data[5]/1000)
            h_list.append(h)

        time_list.append(data.data[2])
        time_read_list.append(data.data[3])
        time_proc_list.append(data.data[4])


    rospy.init_node("pouring_control", anonymous=True)
    rospy.Subscriber("BeerPouring", Float64MultiArray, callback)
    print("running")
    cv2.namedWindow("waiting",cv2.WINDOW_NORMAL)
    cv2.waitKey(0)
    print('plotting')
    t_start = time_abs_list[0]
    map(lambda x: x - t_start, time_abs_list)
    plt.plot(time_abs_list, ratio_list)
    plt.title('beer ratio during pouring')
    plt.xlabel('s')
    plt.figure()
    plt.plot(time_abs_list, h_list)
    plt.title('cup height change during pouring')
    plt.xlabel('s')
    plt.ylabel('m')
    plt.figure()
    plt.plot(time_abs_list, liquid_level_list)
    plt.title('liquid level during pouring')
    plt.xlabel('s')
    plt.ylabel('cm')
    plt.figure()
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
    data_save.append(ratio_list)
    data_save.append(h_list)
    data_save.append(liquid_level_list)
    data_save = np.array(data_save)
    data_save = data_save.T
    save = pd.DataFrame(data_save, columns=['time', 'ratio', 'height', 'liquid_level'])
    doc_dir = '/home/liangxiao/Documents/experiment/'
    data_name = raw_input('input the name of data, including .csv')
    save.to_csv(doc_dir+data_name, index=False, header=False)

