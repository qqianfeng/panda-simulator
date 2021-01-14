// Subscribes to a single joint state command and splits to individual robot joint states- panda and the remaining to the hithand namespace

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Char.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <string>

using namespace std;
ros::Publisher arm_pub,hand_pub;
bool publishers_init=false,split_names_init=false;
sensor_msgs::JointState new_js;
vector<int> arm_indices,hand_indices;
string arm_name;

void stateCallback(sensor_msgs::JointState js)
{
  if(publishers_init)
    {
      // Split arm and hand joint indices
      if(!split_names_init)
	{
	  for(int i=0;i<js.name.size();i++)
	    {
	      if(!js.name[i].find(arm_name))
		{
		  arm_indices.push_back(i);
		}
	      else
		{
		  hand_indices.push_back(i);
		}
	    }

	  split_names_init=true;
	}
      // Publish arm states
      sensor_msgs::JointState arm_states;
      
      arm_states.header=js.header;
      for(int k=0;k<arm_indices.size();k++)
	{
	  int index=arm_indices[k];
	  arm_states.name.push_back(js.name[index]);
	  arm_states.position.push_back(js.position[index]);
	  arm_states.velocity.push_back(js.velocity[index]);
	  arm_states.effort.push_back(js.effort[index]);
	}
      arm_pub.publish(arm_states);
      
      //Publish hand states
      sensor_msgs::JointState hand_states;
      
      hand_states.header=js.header;
      for(int k=0;k<hand_indices.size();k++)
	{
	  int index=hand_indices[k];
	  hand_states.name.push_back(js.name[index]);
	  hand_states.position.push_back(js.position[index]);
	  hand_states.velocity.push_back(js.velocity[index]);
	  hand_states.effort.push_back(js.effort[index]);
	}
      hand_pub.publish(hand_states);
    }
}

/**
 * Starts up a node and initializes controllers
 *
 * robot name
 *
 * @return
 */
int main(int argc,char* argv[])
{
  bool set_robot_name=false;

  // Initialize the node
  ros::init(argc,argv,"robot_joint_splitter");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  std::string listen_prefix;
  std::string publish_prefix;
  std::string hand_prefix;
  n_private.param<std::string>("listen_prefix", listen_prefix,"/panda_hithand");
  n_private.param<std::string>("hand_prefix", hand_prefix,"/hithand");
  n_private.param<std::string>("arm_string",arm_name,"panda");
  // Setup subscriber
  stringstream topic_name;
  topic_name << listen_prefix << "/joint_states";
  ros::Subscriber joint_state_sub=n.subscribe(topic_name.str(), 100, stateCallback);

  // Empty contents of the string buffer
  topic_name.str("");
  topic_name<<hand_prefix<<"/joint_states";


  arm_pub=n.advertise<sensor_msgs::JointState>("/panda/joint_states",1);
  hand_pub=n.advertise<sensor_msgs::JointState>(topic_name.str(),1);
  publishers_init=true;
  // Change the loop rate as required
  int loop_rate_int = 100;
  n_private.getParam("joint_state_loop_rate", loop_rate_int);
  ros::Rate loop_rate(loop_rate_int);
  ROS_INFO_STREAM("Joint state rate set to " << loop_rate_int);
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
