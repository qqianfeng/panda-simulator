#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "init_panda_pose");
    ros::NodeHandle n;
    // Instantiate subscribers for each joint not supposed to be zero
    ros::Publisher j1_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j1_position_controller/command", 100);
    ros::Publisher j2_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j2_position_controller/command", 100);
    ros::Publisher j3_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j3_position_controller/command", 100);
    ros::Publisher j4_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j4_position_controller/command", 100);
    ros::Publisher j5_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j5_position_controller/command", 100);
    ros::Publisher j6_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j6_position_controller/command", 100);
    ros::Publisher j7_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j7_position_controller/command", 100);

    ros::Publisher thumb0_pub = n.advertise<std_msgs::Float64>("panda_hithand/thumb0_position_controller/command", 100);

    // ROS setting
    ros::Rate loop_rate(1);
    int count = 0;
    // Initial pose
    std_msgs::Float64 j1_msg;
    j1_msg.data = 0;
    std_msgs::Float64 j2_msg;
    j2_msg.data = 0;
    std_msgs::Float64 j3_msg;
    j3_msg.data = 0;
    std_msgs::Float64 j4_msg;
    j4_msg.data = -1;
    std_msgs::Float64 j5_msg;
    j5_msg.data = 0;
    std_msgs::Float64 j6_msg;
    j6_msg.data =  1.98847;
    std_msgs::Float64 j7_msg;
    j7_msg.data = -1.57;

    std_msgs::Float64 thumb0_msg;
    thumb0_msg.data = 0.0;

    while(ros::ok() && count < 1){
    ros::Duration(4).sleep();
    j6_pub.publish(j6_msg);
    ros::Duration(1).sleep();
    j1_pub.publish(j1_msg);
    ros::Duration(1).sleep();
	j2_pub.publish(j2_msg);
    ros::Duration(1).sleep();
    j3_pub.publish(j3_msg);
    ros::Duration(1).sleep();
    j4_pub.publish(j4_msg);
    ros::Duration(1).sleep();
    j5_pub.publish(j5_msg);
    ros::Duration(1).sleep();
    j7_pub.publish(j7_msg);
    ros::Duration(1).sleep();
    thumb0_pub.publish(thumb0_msg);
    //ros::spinOnce();
    //loop_rate.sleep();
    ++count;
    }

    return 0;
}