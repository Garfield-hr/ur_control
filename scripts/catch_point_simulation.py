from ring_pre_catch_mini import *
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class UrRobot(MyRobotPlanner):

    def __init__(self, robot_location):
        topic_command = '/arm_controller/command'
        topic_state = '/arm_controller/state'
        control_mode = ControlMode.ikfast
        super(UrRobot, self).__init__(topic_command=topic_command, topic_state=topic_state, control_mode=control_mode)
        self.robot_init_state = [0, -2.936315136835836, 1.769998306768569, -1.975275823522433, 0.143235467068575, 0]
        self.robot.group.go(self.robot_init_state)
        self.traj_para = []
        self.robot_location = robot_location

    def get_current_position(self, t):
        current_point = JointTrajectoryPoint()
        current_point.positions = self.robot_init_state
        if self.traj_para:
            current_point = point_interpolation_fifth(self.traj_para, t)
            self.robot.group.go(current_point.positions, wait=True)
            print('moved', current_point)
        position = self.robot.group.get_current_pose().pose.position
        return [position.x + self.robot_location[0], position.y + self.robot_location[1], position.z + self.robot_location[2]], current_point

    def updata_trajectory(self, traj_para):
        self.traj_para = traj_para



def main():
    rospy.init_node('my_controller', anonymous=True, disable_signals=True)

    # load data
    dir_str = '/home/hairui/Documents/lab/data/12-24-data-20201225T052959Z-001/12-24-data/'
    camera_data = np.load(dir_str + 'camera_result-hr1.npy')
    time_data = np.load(dir_str + 'time_result-hr1.npy')
    print(camera_data.shape)
    print(time_data.shape)


    # set robot parameter
    robot_location = (0.7, 0.0, 0.0)
    robot_reach = 1.0
    robot = UrRobot(robot_location)
    catch_points_x = []
    catch_points_y = []
    catch_points_z = []
    robot_position_x = []
    robot_position_y = []
    robot_position_z = []
    joint0 = []
    joint1 = []
    joint2 = []
    joint3 = []
    joint4 = []
    joint5 = []
    joint_t = []
    success = 0
    # init_orientation = robot.robot.group.get_current_pose().pose.orientation

    # use first i data to calculate catch point


    step=8
    tcatch = 0
    traj_para = []
    print(time_data[0])
    for i in range(0, len(time_data)-1, step):
        print('i=---------------------------')
        print(i)
        theta = get_pre_param2(camera_data[:i+1, :3], time_data[:i+1])  # fitting,
        # theta is the parameter of fitting result
        t1 = solve_time_period2(theta=theta, robot_loc=robot_location, robot_range=robot_reach, zcatch=0.6)
        # intersection with plane z=0.6
        t2 = solve_time_period3(theta=theta, robot_loc=robot_location, robot_range=robot_reach, catch_ratio=0.9)
        # intersection with sphere r=0.9
        if t1 == -1 or t2 == -1 or (t1 < t2):
            print ('cannot catch')
        else:
            # calculate catch point and save it
            if not i:
                present_time = 0
            else:
                present_time = (time_data[i] - time_data[i - step])
            # print('present time is', present_time)
            # print('time now', time_data[i])
            # print ('time before', time_data[i - step])
            position, joints_values = robot.get_current_position(present_time)
            joints_position = joints_values.positions
            joint_t.append(time_data[i])
            joint0.append(joints_position[0])
            joint1.append(joints_position[1])
            joint2.append(joints_position[2])
            joint3.append(joints_position[3])
            joint4.append(joints_position[4])
            joint5.append(joints_position[5])

            robot_position_x.append(position[0])
            robot_position_y.append(position[1])
            robot_position_z.append(position[2])

            #tcatch = solve_time_period2(theta=theta, robot_loc=robot_location, robot_range=robot_reach, zcatch=0.77)
            tcatch, _ = catch_point_least_cartesian_distance(t1, t2, theta, position)
            #tcatch = 0.5*(t1+t2)
            # if catch_points_z:
            #     tcatch, _ = catch_point_least_cartesian_distance(t1, t2, theta, [catch_points_x[-1], catch_points_y[-1],
            #                                                                      catch_points_z[-1]])
            # else:
            #     tcatch, _ = catch_point_least_cartesian_distance(t1, t2, theta, position)
            catch_point = time_to_loc(theta, tcatch)

            # calculate robot trajectory
            start_point = joints_values
            # start_point.positions = joints_position
            complete_point(start_point)
            start_point.time_from_start = rospy.Duration.from_sec(0)
            goal_pose = Pose()
            goal_pose.position.x = catch_point[0] - robot_location[0]
            goal_pose.position.y = catch_point[1] - robot_location[1]
            goal_pose.position.z = catch_point[2] - robot_location[2]
            goal_pose.orientation.x = 0
            goal_pose.orientation.y = 1
            goal_pose.orientation.z = 0
            goal_pose.orientation.w = 0
            # print('goal pose is', goal_pose)
            goal_point_ik_joint_space = ur5e_ik_fast(goal_pose)
            # print("solutions are")
            # for solution in goal_point_ik_joint_space:
            #     print(solution)
            if not goal_point_ik_joint_space:
                print("out of range")
                continue
            best_solution = best_ik_solution(start_point.positions, goal_point_ik_joint_space)
            goal_point = JointTrajectoryPoint()
            goal_point.positions = best_solution
            complete_point(goal_point)
            goal_point.time_from_start = rospy.Duration.from_sec(tcatch - time_data[i])
            _, traj_para = traj_generate_with_two_points(start_point, goal_point)
            robot.updata_trajectory(traj_para)
            catch_points_x.append(catch_point[0])
            catch_points_y.append(catch_point[1])
            catch_points_z.append(catch_point[2])
            success += 1

    time_end = time_data[-1]
    time_step = step*time_end/len(time_data)
    time_catch_num = int(len(time_data)*(tcatch - time_end)/time_end)/step + 1
    time_left = [t*time_step for t in range(1, time_catch_num)]
    for t in time_left:
        position, joints_values = robot.get_current_position(t)
        joints_position = joints_values.positions
        joint_t.append(t + time_end)
        joint0.append(joints_position[0])
        joint1.append(joints_position[1])
        joint2.append(joints_position[2])
        joint3.append(joints_position[3])
        joint4.append(joints_position[4])
        joint5.append(joints_position[5])
        robot_position_x.append(position[0])
        robot_position_y.append(position[1])
        robot_position_z.append(position[2])

    fig = plt.figure()
    ax1 = Axes3D(fig)
    print (success)
    ax1.scatter(catch_points_x, catch_points_y, catch_points_z)
    ax1.scatter(robot_position_x, robot_position_y, robot_position_z)
    ax1.plot3D(catch_points_x, catch_points_y, catch_points_z)
    ax1.plot3D(robot_position_x, robot_position_y, robot_position_z)
    fig2 = plt.figure()
    plt.subplot(321)
    plt.plot(joint_t, joint0)
    plt.subplot(322)
    plt.plot(joint_t, joint1)
    plt.subplot(323)
    plt.plot(joint_t, joint2)
    plt.subplot(324)
    plt.plot(joint_t, joint3)
    plt.subplot(325)
    plt.plot(joint_t, joint4)
    plt.subplot(326)
    plt.plot(joint_t, joint5)
    plt.show()



def test():
    dir_str = '/home/hairui/Documents/lab/data/12-24-data-20201225T052959Z-001/12-24-data/'
    camera_data = np.load(dir_str + 'camera_result-hr-perfect.npy')
    time_data = np.load(dir_str + 'time_result-hr-perfect.npy')
    fig = plt.figure()
    ax1 = Axes3D(fig)
    ax1.plot(camera_data[:, 0], camera_data[:, 1], camera_data[:, 2],color='blue')





    # a0,a1,b0,b1,c0,c1,fac_x,fac_y,fac_z=-2.2,2.5,0,-0.3,1.3,5.5,0.002,0.002,0.005
    # time_sim=np.arange(0.0,0.8,0.001)
    # w, A1, phi1, C1, A2, phi2, C2, A3, phi3, C3 = pi * 4, 0.5, pi * 0.2, 0.1, 0.5, pi * 0.5, 0.5, 0.7, pi * 0.3, -0.3
    # nx = A1 * np.sin(w * time_sim+phi1)+C1
    # ny = A2 * np.sin(w * time_sim + phi2) + C2
    # nz = A3 * np.sin(w * time_sim + phi3) + C3
    # norm=np.sqrt(nx**2+ny**2+nz**2)
    # nx=(nx/norm).reshape(-1,1)
    # ny = (ny / norm).reshape(-1, 1)
    # nz= (nz / norm).reshape(-1, 1)

    # X_sim=a0+a1*time_sim+(np.random.randn(len(time_sim))-0.5)*fac_x
    # Y_sim=b0+b1*time_sim+(np.random.randn(len(time_sim))-0.5)*fac_y
    # Z_sim=c0+c1*time_sim-4.9*time_sim**2+(np.random.randn(len(time_sim))-0.5)*fac_z
    # X_sim=X_sim[200:]
    # Y_sim=Y_sim[200:]
    # Z_sim=Z_sim[200:]
    # T_sim=time_sim[200:]

    a0, a1, b0, b1, c0, c1, fac_x, fac_y, fac_z = -2.2, 2.5, 0, -0.3, 1.3, 5.5, 0.002, 0.002, 0.005


    w, A1, phi1, C1, A2, phi2, C2, A3, phi3, C3 = pi * 6, 0.9, pi, 0.0, 0.1, pi * 0.5, 0.0, 0.9, pi * 2, -0.0
    orientation_param = [w, A1, phi1, C1, A2, phi2, C2, A3, phi3, C3]

    time_sim = np.arange(0.0, 0.8, 0.001)
    X_sim = a0 + a1 * time_sim + (np.random.randn(len(time_sim)) - 0.5) * fac_x
    Y_sim = b0 + b1 * time_sim + (np.random.randn(len(time_sim)) - 0.5) * fac_y
    Z_sim = c0 + c1 * time_sim - 4.9 * time_sim ** 2 + (np.random.randn(len(time_sim)) - 0.5) * fac_z
    nx_sim = A1 * np.sin(w * time_sim+phi1)+C1
    ny_sim = A2 * np.sin(w * time_sim + phi2) + C2
    nz_sim = A3 * np.sin(w * time_sim + phi3) + C3
    norm=np.sqrt(nx_sim**2+ny_sim**2+nz_sim**2)
    nx_sim=(nx_sim/norm)
    ny_sim = (ny_sim / norm)
    nz_sim= (nz_sim / norm)
    nx_sim[np.where(nz_sim>0)]= -nx_sim[np.where(nz_sim> 0)]
    ny_sim[np.where(nz_sim> 0)] = -ny_sim[np.where(nz_sim> 0)]
    nz_sim[np.where(nz_sim>0)] = -nz_sim[np.where(nz_sim >0)]
    X_sim = X_sim[200:]
    Y_sim = Y_sim[200:]
    Z_sim = Z_sim[200:]
    nx_sim= nx_sim[200:]
    ny_sim = ny_sim[200:]
    nz_sim = nz_sim[200:]
    T_sim = time_sim[200:]

    print(X_sim.shape)
    print(nx_sim.shape)

    ax1.plot(X_sim, Y_sim, Z_sim, color='red')
    ax1.quiver(X_sim[::5], Y_sim[::5], Z_sim[::5], nx_sim[::5], ny_sim[::5], nz_sim[::5],arrow_length_ratio=0.1,length=0.1,normalize=True,color='green')

    plt.axis("equal")
    # my_x_ticks = np.arange(-4, 4, 0.5)
    # my_y_ticks = np.arange(-1, 1, 0.5)
    # my_z_ticks = np.arange(1, 4, 0.5)
    # ax1.set_xticks(my_x_ticks)
    # ax1.set_yticks(my_y_ticks)
    # ax1.set_zticks(my_z_ticks)

    plt.show()



def calculate_catch_point(para, method):
    methods = {
        'plane': calculate_catch_point_plane
    }
    if methods in methods:
        return methods[method](para)

def calculate_catch_point_plane(para):
    return solve_time_period2(theta=para[0], robot_loc=para[1], robot_range=para[2], zcatch=para[3])

if __name__ == '__main__':
    main()
    #test()
