import time

import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import cv2

def callback1(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard")

if __name__ == '__main__':
    ratio_list = []
    liquid_level_list = []
    time_list = []
    def callback(data):
        if data.data[0] > 1e-3:
            ratio_list.append(data.data[0])
            liquid_level_list.append(data.data[1])
        time_list.append(data.data[2])

    rospy.init_node("pouring_control", anonymous=True)
    rospy.Subscriber("BeerPouring", Float64MultiArray, callback)
    print("running")
    cv2.namedWindow("waiting",cv2.WINDOW_NORMAL)
    cv2.waitKey(0)
    print('plotting')
    plt.plot(ratio_list)
    plt.title('beer ratio during pouring')
    plt.xlabel('frame')
    plt.figure()
    plt.plot(liquid_level_list)
    plt.title('liquid level during pouring')
    plt.xlabel('frame')
    plt.ylabel('cm')
    plt.figure()
    plt.plot(time_list)
    plt.title('time consumed during processing')
    plt.xlabel('frame')
    plt.ylabel('ms')
    plt.show()
