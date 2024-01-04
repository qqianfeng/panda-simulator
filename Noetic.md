System info: Ubuntu 20.04 - gazebo 11 - ROS noetic
# 1. Install CUDA
wsl would need diff setup
https://ubuntu.com/tutorials/enabling-gpu-acceleration-on-ubuntu-on-wsl2-with-the-nvidia-cuda-platform#1-overview

---
# 2. Install ros-gazebo
**Follow the instruction here:
https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros**
## 2.1 Install ROS NOETIC 
**Run `bash install-ros_noetic.sh`**

---
*Reference:*
Desktop-Full Install
https://wiki.ros.org/noetic/Installation/Ubuntu


---
## 2.2 Install Gazebo 11
**Install `curl -sSL http://get.gazebosim.org | sh`**

Run `gazebo`

## 2.3 Install gazebo_ros_pkgs
Run 
```
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

Test

Assuming your ROS and Gazebo environment have been properly setup and built, you should now be able to run Gazebo through a simple rosrun command, after launching roscore if needed:
```
source /opt/ros/noetic/setup.bash

roscore &
rosrun gazebo_ros gazebo
```

# 3. Dependencies
```
pip install -r requirements.txt
sudo apt install -y ros-noetic-libfranka ros-noetic-rospy-message-converter ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-moveit ros-noetic-moveit-commander ros-noetic-moveit-visual-tools

```

1. pip install -r requirements.txt #(to install numpy and numpy-quaternion) (or pip3 install -r requirements.txt)

2. libfranka (apt install ros-${ROS_DISTRO}-libfranka or install from source).

3. Most of the other basic dependencies can be met by running the following apt-get command:
```
apt install ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-visual-tools
```
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
