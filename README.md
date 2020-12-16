  

  

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

  

`source /opt/ros/melodic/setup.sh`

  

`catkin_init_workspace src`

  

- Clone franka_ros from Github and checkout correct version

  

`git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros`


`cd src/franka_ros/`


`git checkout 49e5ac1`

`cd ../../`  

- Install missing dependencies, replace the /path/to/libfranka/build with your libfranka/build directory

  

`rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka`

  

`catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`

- In case of error :  catkin command not found, run: 
`sudo apt-get install python-catkin-tools`

`source devel/setup.sh`

  

#### Other dependencies

-  `sudo apt update && sudo apt install -q -y build-essential git swig sudo python-future libcppunit-dev python-pip`


-  `sudo apt update && sudo apt install -y python-catkin-tools ros-melodic-gazebo-ros-control ros-melodic-rospy-message-converter ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-state-controller python-pip ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools ros-melodic-tf2-sensor-msgs ros-melodic-rosbridge-server ros-melodic-tf2-web-republisher ros-melodic-ros-control ros-melodic-moveit`


-  `pip install --upgrade pip`


-  `sudo apt update && sudo apt upgrade -y`

cd to panda-simulator repo:
-  `pip install -r requirements.txt`

  
  

#### Building the Package

  

1. Clone the repo:

  

```bash

  

cd hithand_ws/src

  

git clone git@git.ar.int:deeplearn/hithand-grasp/panda-simulator.git

  

```

  

2. Update dependency packages. This will install `franka-panda-description`, `franka-ros-interface`, `orocos-kinematic-dynamics` and `gazebo-realsense-plugin` from the Gitlab Hithand group:

  

  

```bash

  

wstool init

  

wstool merge panda-simulator/dependencies.rosinstall

  
  

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

Now you test the control by sending a command to the corresponding controller topic. E.g.

```bash

  

  

rostopic pub /panda_hithand/panda_j1_position_controller/command std_msgs/Float64 "data: 0.0"

  

  

```

## Grasping Pipeline
For the entire grasping pipeline you will need more packages

### Installation
1. Hithand ROS package
- Clone the Hithand ROS Github package into your hithand_ws/src/ directory \
`cd hithand_ws/src`
`git clone https://github.com/vincentmaye/hithand_ros.git`
2. Panda Hithand Moveit Config
- Clone the panda-hithand-moveit package also in the /src folder \
`git clone git@git.ar.int:deeplearn/hithand-grasp/panda-hithand-moveit-config.git` 
3. Grasp pipeline
- Clone the grasp-pipeline package which provides the core grasping client-server functionality. \
`git clone git@git.ar.int:deeplearn/hithand-grasp/grasp-pipeline.git`
- This is actually a bit more effort than the previous packages. First install the python requirements \
`cd grasp-pipeline`
`pip install -r requirements.txt`
- For this package to work you will also have to install anaconda. Follow the instructions listed on this website
https://docs.anaconda.com/anaconda/install/linux/
- Once Anaconda is installed successfully you have to create a conda env called py37 \
`conda create -n py37 python=3.7`
- Activate the conda env and install requirementes \
`conda activate py37`
`pip install -r requirements_py37`
NOTE: If some of the installed packages cause errors, you have to pip uninstall them and use conda instead. Google for how to install the corresponding package using conda to find the correct command.
4. Bashrc Modifications
- For the whole system to work some modifications have to made to your .bashrc file. (This is analogous for other shells). \
`vim ~/.bashrc` \
At the bottom of the file first delete everything added by anaconda (the line is indicated by `#>>>>>> conda initialize` and append the following instead:
NOTE: Make sure to replace any occurences of `/home/vm` with the right base path for your system
	````bash
	# >>> conda initialize >>>                                                             
	# !! Contents within this block are managed by 'conda init' !!                         
	__conda_setup="$('/home/vm/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /\
	dev/null)"
	if [ $? -eq 0 ]; then
	    eval "$__conda_setup"
	#else
	#    if [ -f "/home/vm/anaconda3/etc/profile.d/conda.sh" ]; then
	# . "/home/vm/anaconda3/etc/profile.d/conda.sh"  # commented out by conda initialize
	#    else
	# export PATH="/home/vm/anaconda3/bin:$PATH"  # commented out by conda initialize
	#    fi
	fi
	unset __conda_setup
	# <<< conda initialize <<<

	source /opt/ros/melodic/setup.bash
	source /home/vm/hithand_ws/devel/setup.bash
	export LC_NUMERIC="en_US.UTF-8"
	export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}] [${logger}]: ${message}'

	export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/vm/object_datasets/ycb/models
	````

### Starting Procedure
The whole system gets started in the following order. Don't be too quick with executing the commands below and execute each of them in a seperate terminal.
1. Start the panda_simulator \
`roslaunch panda_gazebo panda.launch` 
2. Start the panda_hithand_moveit_config \
`roslaunch panda_hithand_moveit_config panda_hithand_moveit.launch` 
3. Start the grasp_pipeline. This exposes the the grasping servers. Currently this does not do anything in and of itself. But you can for example spawn objects in Gazebo \ 
`roslaunch grasp_pipeline grasp_pipeline_servers.launch` 