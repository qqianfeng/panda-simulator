# Install Gazebo 11 with dart from source
https://classic.gazebosim.org/tutorials?tut=install_from_source&cat=install

Notice: this file is missing when you try to make -j4, download it manually and copy-paste to the correct path would solve the problem.

https://github.com/dartsim/dart/blob/release-6.13/dart/utils/detail/XmlHelpers-impl.hpp

# Install CUDA

# Install ROS NOETIC 
Desktop-Full Install
http://wiki.ros.org/noetic/Installation/Ubuntu

One or two - liner Installation
http://wiki.ros.org/ROS/Installation/TwoLineInstall/

Note: choose the Desktop Install (NO Desktop-Full Install!!)

# Dependencies
1. pip install -r requirements.txt #(to install numpy and numpy-quaternion) (or pip3 install -r requirements.txt)

2. libfranka (apt install ros-${ROS_DISTRO}-libfranka or install from source).

3. Most of the other basic dependencies can be met by running the following apt-get command: apt install ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-visual-tools

4. sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
ros-noetic-image-common

Note: 
Problem 1 - ros-$ROS_DISTRO-gazebo-ros-control: 
dependency problem with gazebo_ros_pkgs
Solution: install from source
https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros


# if not running 
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/hithand_ws/devel/setup.bash
