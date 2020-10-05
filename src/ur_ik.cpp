//
// Created by hairuizhu on 2020/10/02.
//
#include "ur_ik.h"
#include <boost/bind.hpp>

urIkNode::urIkNode(const string& pub_topic, const string& sub_topic,
                   const string& robot_description)
                   : robot_model_loader(robot_description),
                     kinematic_model(robot_model_loader.getModel()),
                     kinematic_state(new robot_state::RobotState(kinematic_model)){
    pub = n.advertise
            <trajectory_msgs::JointTrajectoryPoint>
            (pub_topic, 10);
    sub = n.subscribe(sub_topic, 10,
            &urIkNode::transfer, this);

}

urIkNode::~urIkNode(){}

void urIkNode::transfer(const geometry_msgs::Pose::ConstPtr& eeState) {
    const robot_state::JointModelGroup *joint_model_group =
            kinematic_model->getJointModelGroup("manipulator");
    Eigen::AngleAxisd rotation_vector( eeState->orientation.w,
            Eigen::Vector3d( eeState->orientation.x,
                    eeState->orientation.y, eeState->orientation.z));
    cout<<rotation_vector.toRotationMatrix()<<endl;
    Eigen::Isometry3d transformMat = Eigen::Isometry3d::Identity();
    transformMat.rotate(rotation_vector);
    transformMat.pretranslate(Eigen::Vector3d(eeState->position.x,
                                           eeState->position.y, eeState->position.z));
    cout<<transformMat.matrix()<<endl;
    double timeout = 0.01;
    //clock_t start, end;
    //start = clock();

    bool found_ik = kinematic_state->setFromIK(joint_model_group, transformMat, timeout);
    //end = clock();

    if (found_ik)
    {
        const double time_to_goal = 0.2;
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        trajectory_msgs::JointTrajectoryPoint goalPoint;
        goalPoint.positions = joint_values;
        goalPoint.time_from_start = ros::Duration(time_to_goal);
        pub.publish(goalPoint);
    }

}

