//
// Created by hairuizhu on 2020/10/02.
//



// MoveIt
#include "ur_ik.h"
#include <boost/bind.hpp>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_ur_ik");
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

    const string pub_topic = "my_command";
    const string sub_topic = "catch_point";
    const string robot_description = "robot_description";

    urIkNode ik(pub_topic, sub_topic, robot_description);

    ros::spin();
    return 0;

}

