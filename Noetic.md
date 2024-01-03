System info: Ubuntu 20.04 - gazebo 9 - ROS noetic
# 1. Install CUDA
wsl would need diff setup
https://ubuntu.com/tutorials/enabling-gpu-acceleration-on-ubuntu-on-wsl2-with-the-nvidia-cuda-platform#1-overview

---
# https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
# 2. Install ROS NOETIC 
**Run `bash install-ros_noetic.sh`**

---
*Reference:*
Desktop-Full Install
http://wiki.ros.org/noetic/Installation/Ubuntu

Note: choose the Desktop Install (NO Desktop-Full Install!!)

---
# 3. Install Gazebo 9 with dart from source
**Run `bash install-gazebo.sh`**

**There will be an error for not finding XmlHelpers-impl.hpp file.**

*Root Cause: Dart project supports official only till ubuntu 19.04.*

Please download and copy https://github.com/dartsim/dart/blob/release-6.13/dart/utils/detail/XmlHelpers-impl.hpp to the path 
/usr/include/dart/utils/detail/XmlHelpers-impl.hpp, and then run 
`make -j4` under /tmp/gazebo/build again.

In the end, run `sudo make install`

Path setup 
```
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
sudo ldconfig
```

For wsl, libcuda needs to adjusted: https://github.com/microsoft/WSL/issues/5548

---
*Reference:*

gazebo.sh is built based on the following installation guide for convenience: https://classic.gazebosim.org/tutorials?tut=install_from_source&cat=install
(Section "Prerequisites", "Install Required Dependencies", "Optional Physics Engines", "Build And Install Gazebo"(step 1-8).)

---

# 4. Dependencies
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
