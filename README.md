
  

# Panda Simulator

  

A **Gazebo simulator** for the Franka Emika Panda robot with ROS interface with the option to attach a DLR-HIT Hand II as end-effector, providing exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the [*franka-ros*][franka-ros] package.

  
  

## Features

  
  

- Low-level *controllers* (joint position, velocity, torque) available that can be controlled through ROS topics (including position control for gripper) or [Python API][fri-repo].

  

- Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics.

  

- The [*Franka ROS Interface*][fri-repo] package (which is a ROS interface for controlling the real Panda robot) and [PandaRobot][https://github.com/justagist/panda_robot] Python API can also be used with the panda_simulator, providing kinematics and dynamics computation for the robot, and direct *sim-to-real* code transfer.

  

- Supports MoveIt planning and control for Franka Panda Emika robot and arm and Franka Gripper.

  

  

## Installation

  

  

#### Installing Franka Emika Software

  

1. Install and build libfranka v0.6.0 from source

  

- Remove any existing libfranka

	`sudo apt remove "*libfranka*"`

- Install dependencies

	`sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`

- Clone the source code

	`git clone --recursive https://github.com/frankaemika/libfranka`

	`cd libfranka`

- Git checkout version 0.6.0

	`git checkout 2b99ab9`

	`git submodule update`

- In the source directory, create a build directory and run CMake

	`mkdir build`

	`cd build`

	`cmake -DCMAKE_BUILD_TYPE=Release ..`

	`cmake --build .`

2. Install and build Franka-ROS v0.6.0

- Create a catkin workspace

	`cd /path/to/desired/folder`

	`mkdir -p hithand_ws/src`

	`cd hithand_ws`

	`source /opt/ros/melodic/steup.sh`

	`catkin_init_workspace src`

- Clone franka_ros from Github and checkout correct version

	`git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros`

	`git checkout 49e5ac1`

- Install missing dependencies, replace the /path/to/libfranka/build with your libfranka/build directory

	`rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka`

	`catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`

	`source devel/setup.sh`

#### Other dependencies
- `sudo apt update && apt install -q -y build-essential git swig sudo python-future libcppunit-dev python-pip`
- `sudo apt update && apt install -y python-catkin-tools ros-melodic-gazebo-ros-control ros-melodic-rospy-message-converter ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-state-controller python-pip ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools`
- `pip install --upgrade pip`
- `sudo apt update && apt upgrade -y`
- `pip install -r requirements.txt`


#### Building the Package

1. Clone the repo:

	```bash

	cd hithand_ws/src
  

	git clone git@git.ar.int:deeplearn/hithand-grasp/panda-simulator.git

  
	```
 

2. Update dependency packages:

  

	```bash

	 
	wstool init

	  
	wstool merge panda_simulator/dependencies.rosinstall


	wstool up

	 

	# use old ros-compatible version of kdl

	  

	cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc

	  

	cd ../.. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

	  

	```


3. Once the dependencies are met, the package can be installed using catkin build (preferred over catkin_make):


	```bash

	  

	source /opt/ros/$ROS_DISTRO/setup.bash

	  

	catkin build # if catkin not found, install catkin tools (apt install python-catkin-tools)

	  

	source devel/setup.bash

	  

	```

 
### Usage

  

  

The simulator can be started by running:

  

  

```bash

  

roslaunch panda_gazebo panda.launch

  

```