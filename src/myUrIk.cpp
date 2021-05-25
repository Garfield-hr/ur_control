//
// Created by hairuizhu on 2020/10/02.
//



// MoveIt
#include "ur_ik.h"
#include <boost/bind.hpp>
#include <fstream>
#include <opencv2/core.hpp>


void ur5e_workspace();

int main(int argc, char **argv)
{
      ros::init(argc, argv, "my_ur_ik");
//    // ros::AsyncSpinner spinner(1);
//    // spinner.start();
//
//    const string pub_topic = "my_command";
//    const string sub_topic = "catch_point";
//    const string robot_description = "robot_description";
//
//    urIkNode ik(pub_topic, sub_topic, robot_description);
//
//    ros::spin();
    ur5e_workspace();
    return 0;
}

void ur5e_workspace()
{
    ofstream outfile;
    outfile.open("/home/hairui/catkin_ws/src/my_ur_control/doc/ur5e_workspace.csv", ios::out | ios::trunc );
    outfile<<"x"<<","<<"y"<<","<<"z"<<endl;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    for (int i=0; i<10000; i++)
    {
        kinematic_state->setToRandomPositions(joint_model_group);
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");
        auto translation = end_effector_state.translation();
        outfile<<translation.x()<<","<<translation.y()<<","<<translation.z()<<endl;

    }
    outfile.close();

}

