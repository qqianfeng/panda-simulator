System info: Ubuntu 20.04 - gazebo 11 - ROS noetic
# 1. Install CUDA
wsl would need diff setup
https://ubuntu.com/tutorials/enabling-gpu-acceleration-on-ubuntu-on-wsl2-with-the-nvidia-cuda-platform#1-overview

---
# 2. Install ros-gazebo
**Reference:
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

# 3. Install franka-panda-simulator
## 3.1 dependencies
```
pip install -r requirements.txt

sudo apt install ros-noetic-libfranka

sudo apt install -y ros-noetic-libfranka ros-noetic-rospy-message-converter ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-moveit ros-noetic-moveit-commander ros-noetic-moveit-visual-tools

```

## 3.2 install
The following dependencies can be installed using the .rosinstall file (instructions in next section: Building the Package).

franka-ros
panda_moveit_config
Franka ROS Interface (branch v0.7.1-dev branch)
franka_panda_description (urdf and model files from panda_description package modified to work in Gazebo, with the custom controllers, and more realistic dynamics parameters)
orocos-kinematics-dynamics (requires a specific commit; see instructions below)

NOTE: The franka_panda_description package above has to be independently updated regularly (using git pull) to get the latest robot description, visual and dynamics parameters.

```
cd <catkin_ws>/src
git clone -b noetic-hithand https://github.com/qianbot/panda-simulator.git

wstool init
wstool merge panda_simulator/dependencies.rosinstall
wstool up

# use old ros-compatible version of kdl
cd orocos_kinematics_dynamics && rm -rf * && git checkout b35c424e && git reset --hard
cd ../../.. && rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys python-sip

source /opt/ros/noetic/setup.bash
catkin build # if catkin not found, install catkin tools (apt install python3-catkin-tools)

echo "source ~/hithand_ws/devel/setup.bash" >> ~/.bashrc
source devel/setup.bash
```

## 3.3 test
```
roslaunch panda_gazebo panda_world.launch
```