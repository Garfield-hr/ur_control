//
// Created by hairuizhu on 2020/10/02.
//

#ifndef MY_UR_CONTROL_UR_IK_H
#define MY_UR_CONTROL_UR_IK_H

#endif //MY_UR_CONTROL_UR_IK_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace  std;

class urIkNode
{
public:
    urIkNode(const string& pub_topic, const string& sub_topic,
            const string& robot_description);
    ~urIkNode();
    void transfer(const geometry_msgs::Pose::ConstPtr& eeState);
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
};
