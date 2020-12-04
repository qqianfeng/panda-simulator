#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "init_panda_pose");
    ros::NodeHandle n;
    // Instantiate subscribers for each joint not supposed to be zero
    ros::Publisher j1_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j1_position_controller/command", 1000);
    ros::Publisher j2_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j2_position_controller/command", 1000);
    ros::Publisher j4_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j4_position_controller/command", 1000);
    ros::Publisher j6_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j6_position_controller/command", 1000);
    ros::Publisher j7_pub = n.advertise<std_msgs::Float64>("panda_hithand/panda_j7_position_controller/command", 1000);
    
    // ROS setting
    ros::Rate loop_rate(1);
    int count = 0;
    // Initial pose
    std_msgs::Float64 j1_msg;
    j1_msg.data = -1.5708;
    std_msgs::Float64 j2_msg;
    j2_msg.data = -0.553876;
    std_msgs::Float64 j4_msg;
    j4_msg.data = -2.5361;
    std_msgs::Float64 j6_msg;
    j6_msg.data =  1.98847;
    std_msgs::Float64 j7_msg;
    j7_msg.data = -0.785;

    while(ros::ok() && count < 5){
	j1_pub.publish(j1_msg);        
	j2_pub.publish(j2_msg);
        j4_pub.publish(j4_msg);
        j6_pub.publish(j6_msg);
        j7_pub.publish(j7_msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
