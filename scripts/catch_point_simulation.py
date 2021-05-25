from ring_pre_catch_mini import *
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    dir_str = '/home/hairui/Documents/lab/data/12-24-data-20201225T052959Z-001/12-24-data/'
    camera_data = np.load(dir_str + 'camera_result-hr1.npy')
    time_data = np.load(dir_str + 'time_result-hr1.npy')
    robot_location = (0.5, 0.0, 0.0)  # ji de gai
    robot_reach = 1.0
    catch_points_x = []
    catch_points_y = []
    catch_points_z = []
    success = 0

    for i in range(len(time_data)):
        theta = get_pre_param2(camera_data[:i+1, :3], time_data[:i+1])
        t1 = solve_time_period2(theta=theta, robot_loc=robot_location, robot_range=robot_reach, zcatch=0.6)
        t2 = solve_time_period3(theta=theta, robot_loc=robot_location, robot_range=robot_reach, catch_ratio=0.9)
    if t1 == -1 or t2 == -1 or (t1 < t2):
        print ('cannot catch')
        catch_points_x.append(0)
        catch_points_y.append(0)
        catch_points_z.append(0)
    else:
        tcatch = solve_time_period2(theta=theta, robot_loc=robot_location, robot_range=robot_reach, zcatch=0.77)
        if (tcatch<t1) and (tcatch>t2):
            catch_point = time_to_loc(theta, tcatch)
            catch_points_x.append(catch_point[0])
            catch_points_y.append(catch_point[1])
            catch_points_z.append(catch_point[2])
            success += 1


    fig = plt.figure()
    ax1 = Axes3D(fig)
    print (success)
    ax1.plot3D(catch_points_x, catch_points_y, catch_points_z)
    plt.show()


def test():
    dir_str = '/home/hairui/Documents/lab/data/12-24-data-20201225T052959Z-001/12-24-data/'
    camera_data = np.load(dir_str + 'camera_result-hr1.npy')
    time_data = np.load(dir_str + 'time_result-hr1.npy')
    fig = plt.figure()
    ax1 = Axes3D(fig)
    ax1.plot3D(camera_data[:, 0], camera_data[:, 1], camera_data[:, 2])
    plt.show()


if __name__ == '__main__':
    main()