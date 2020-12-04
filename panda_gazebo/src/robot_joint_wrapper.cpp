// Subscribes to a single joint state command and publishes the data to individual joints

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
bool got_joints_=false, publishers_init_=false, got_control_method_=false, notified_user_=false;
int n_joints_;
char control_method_;
map<string, ros::Publisher> joint_pub_;
vector<string> joint_names_;

vector<string> panda_joint_names {"panda_j1", "panda_j2", "panda_j3", "panda_j4", "panda_j5", "panda_j6", "panda_j7"};
vector<string> hithand_joint_names {"thumb1", "thumb2", "thumb3", "thumb4", "index1", "index2", "index3", "index4",  "middle1",  "middle2",  "middle3",  "middle4",  "ring1",  "ring2",  "ring3",  "ring4",  "little1",  "little2",  "little3",  "little4",};

// Listen to: /hithand/joint_cmd , then split this command and publish to /hithand/middle1_position_controller/command

/**
 * Check if a commanded control mode is valid
 *
 * @param control_mode The control mode
 *
 * @return true if valid, false otherwise
 */
bool controlArgValid(char control_mode)
{
  if(control_mode =='p' || control_mode =='v' || control_mode =='e')
  {
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[RobotJointWrapper] Unknown control method:" << control_mode
		     << " publish control_method as 'p' for position, "
		     << "'v' for velocity or 'e' for effort control");
    return false;
  }
}

/**
 * Callback function for joint commands. Processes joint command message and publishes
 * to underlynig joint topics.
 *
 * @param jc joint command message
 */
void jointCallback(sensor_msgs::JointState jc)
{
  if(got_control_method_)
  {
    bool command_size_correct;
    vector<double> position=jc.position;
    vector<double> velocity=jc.velocity;
    vector<double> effort=jc.effort;
    if((position.size() == n_joints_) || (velocity.size() == n_joints_) || (effort.size() == n_joints_)){
      command_size_correct = true;
    }
    if(!command_size_correct){
      ROS_ERROR_STREAM("The command had the wrong size. Size of command should be equal to number of controlled joints:" << n_joints_);
    }
    if(publishers_init_)
    {
      for(int i=0;i<n_joints_;i++)
      {
        std_msgs::Float64 joint_data;
        if(control_method_ == 'p')
        {
          joint_data.data=position[i];
        }
        else if(control_method_ == 'v')
        {
          joint_data.data=velocity[i];
        }
        else if(control_method_ == 'e')
        {
          joint_data.data=effort[i];
        }
        else
        {
          ROS_ERROR_STREAM("[RobotJointWrapper] Unknown control method:" << control_method_
                           << " publish control_method as 'p' for position,"
			   << "'v' for velocity or 'e' for effort control");
        }
        //Publish joint data
        joint_pub_[joint_names_[i]].publish(joint_data);
      }
    } else
    {
      // This is so you you don't continually yell at the user, which happens in Orocos sim
      if (!notified_user_)
      {
	      ROS_INFO("[RobotJointWrapper] Need to initialize publishers!");
	      notified_user_ = true;
      }
    }
  }
}


/**
 * Callback function for setting the control mode
 *
 * @param c_method char message defining the control mode between
 * position ('p'), velocity ('v'), and effort ('e')
 */
void controlCallback(std_msgs::Char c_method)
{
  if( ! controlArgValid(c_method.data) ) return;
  control_method_ = c_method.data;
  got_control_method_ = true;
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
  ros::init(argc,argv,"robot_joint_wrapper");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  std::string listen_prefix;
  std::string publish_prefix;
  std::string jnt_cmd_topic;

  n_private.param<std::string>("listen_prefix", listen_prefix,"/panda");
  n_private.param<std::string>("publish_prefix", publish_prefix,"/panda"); // PUBLISH: /hithand/JOINT_position_controller/command
  n_private.param<std::string>("jnt_cmd_topic", jnt_cmd_topic, listen_prefix + "/joint_cmd"); // LISTEN: /hithand/joint_cmd

  listen_prefix.insert(0, 1, '/');
  publish_prefix.insert(0, 1, '/');

  //cout << "Listen prefix:" << listen_prefix;
  //cout << "\n Publish prefix:" << publish_prefix;

  // this section just assigns the hard coded panda/hithand joint names
  if(publish_prefix == "/panda_hithand"){
    joint_names_ = panda_joint_names;
    n_joints_ = joint_names_.size();
  }
  else if(publish_prefix == "/hithand"){
    joint_names_ = hithand_joint_names;
    n_joints_ = joint_names_.size();
  }
  else{
    ROS_ERROR_STREAM("Unknown publish prefix, only /panda or /hithand currently available. If using different robot or hand, change this script");
  }
  got_joints_ = true;

  // Setup subscribers
  stringstream topic_name;
  topic_name << listen_prefix << "/control_type";
  ros::Subscriber control_sub=n.subscribe(topic_name.str(), 100, controlCallback); // Change the control type with this

  ros::Subscriber joint_sub=n.subscribe(jnt_cmd_topic, 100, jointCallback); // This listens to the data on /robot/joint_cmd

  // Change the loop rate as required
  int loop_rate_int = 100;
  n_private.getParam("control_loop_rate", loop_rate_int);
  ros::Rate loop_rate(loop_rate_int);
  ROS_INFO_STREAM("[RobotJointWrapper] Control rate set to " << loop_rate_int);
  string control_method;
  n_private.param<string>("control_method", control_method, "p");
  ROS_DEBUG_STREAM("control_method input " << control_method << "\tset to " <<
                   control_method_);

  if (control_method.size() > 0 && controlArgValid(control_method[0]))
  {
    control_method_ = control_method[0];
    got_control_method_ = true;
    ROS_INFO_STREAM("[RobotJointWrapper] Initialized control method to " << control_method_);
  }

  stringstream pub_name;
  while(ros::ok())
  {
    if(got_joints_ && !publishers_init_)
    {
      // Init Publishers
      // joint_pub_.resize(n_joints_);
      for(int i = 0; i < n_joints_; i++)
      {
        pub_name.str(""); // empty buffer
        pub_name << publish_prefix << "/";
        if(control_method_ == 'p')
        {
          pub_name << joint_names_[i] << "_position_controller/command";
        }
        else if(control_method_ == 'v')
        {
          pub_name << joint_names_[i] << "_velocity_controller/command";
        }
        else if(control_method_ == 'e')
        {
          pub_name << joint_names_[i] << "_effort_controller/command";
        }
        else
        {
          ROS_ERROR_STREAM("[RobotJointWrapper] publish control_method as 'p' for position, "
		    << "'v' for velocity or 'e' for effort control");
        }
        joint_pub_[joint_names_[i]]=n.advertise<std_msgs::Float64>(pub_name.str(), 1);
      }
      publishers_init_ = true;
      ROS_INFO_STREAM("Publishers are now INITIALIZED, RECEIVED JOINT NAMES!");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
