  

  

# Panda Simulator

  

  

A **Gazebo simulator** for the Franka Emika Panda robot with ROS interface with the option to attach a DLR-HIT Hand II as end-effector, providing exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the [*franka-ros*][franka-ros] package.

  

  

## Features

  

  

- Low-level *controllers* (joint position, velocity, torque) available that can be controlled through ROS topics (including position control for gripper) or [Python API][fri-repo].

  

  

- Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics.

  

  

- The [*Franka ROS Interface*][fri-repo] package (which is a ROS interface for controlling the real Panda robot) and [PandaRobot][https://github.com/justagist/panda_robot] Python API can also be used with the panda_simulator, providing kinematics and dynamics computation for the robot, and direct *sim-to-real* code transfer.

  

  

- Supports MoveIt planning and control for Franka Panda Emika robot and DLR-HIT hand.

  

  

  

## Installation Process

This section guides through the whole installation process, not only for the panda_hithand_simulator but for the full grasping system. This includes 

- Gazebo 9 from source with DART support.
- ROS melodic.
- libfranka and franka-ros
- wide range of ros-melodic and python packages
- CUDA 10.1 and tensorflow-gpu 1.14.

#### Install Gazebo 9 from source

The first step is to install Gazebo 9 from source. Installing from source is necessary to get full DART support, which is most suitable for grasping applications under all solvers available for Gazebo. Ideally you would have a fresh Ubuntu install or at least make sure you have no Gazebo/ ROS installed on your system as I found this to interfere with the installation process.

NOTE for next step: As installation path, use the recommended /home/$USER/local and after cloning the Gazebo repository be sure to git checkout gazebo9 as building the main branch will not work.

Please follow the steps outlined here CAREFULLY: [Gazebo Installation from Source][http://gazebosim.org/tutorials?tut=install_from_source&cat=install]




#### Install CUDA 10.1 and tensorflow-gpu

In order to run the ML-based grasping inference process (to find a grasping pose, given a point cloud observation) you will need tensorflow which needs CUDA 10.1

Follow the steps closely in this installation script and restart your computer after installation. I would suggest first removing any NVIDIA / CUDA software from your system as I found this to interfere with the installation process.

NOTE for the next step: Instead of executing wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/7.6.5.32/Production/10.1_20191031/cudnn-10.1-linux-x64-v7.6.5.32.tgz you will need to find the that file cudnn-10.1-linux-x64-v7.6.5.32.tgz on NVIDIAs website, after logging in. Running this command will fail, because you need to authorise with NVIDIA first.


[CUDA 10.1 Installation][https://gist.github.com/vincentmaye/090159c493adbf6b3d8f39329c78d12c]

After completing the CUDA installation and verifying via nvidia-smi and nvcc -V proceed with:

pip install tensorflow-gpu==1.14


#### Install ROS melodic

Now follow the instruction from this link:
[ROS melodic Installation][http://wiki.ros.org/melodic/Installation/Ubuntu]


#### Installing Franka Emika Software

  
  

Due to recent updates this step became really easy and works with the binaries of libfranka and franka-ros.

Simply run:
` sudo apt-get install ros-melodic-libfranka ros-melodic-franka-ros`

  

#### Other dependencies

-  `sudo apt-get update && sudo apt-get install -q -y build-essential git swig sudo python-future libcppunit-dev python-pip`


-  `sudo apt-get update && sudo apt-get install -y python-catkin-tools ros-melodic-gazebo-ros-control ros-melodic-rospy-message-converter ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-state-controller python-pip ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools ros-melodic-tf2-sensor-msgs ros-melodic-rosbridge-server ros-melodic-tf2-web-republisher ros-melodic-ros-control ros-melodic-moveit ros-melodic-ros-numpy ros-melodic-trac-ik-python ros-melodic-image-proc ros-melodic-image-pipeline`

-  `pip install --upgrade pip`


-  `sudo apt-get update && sudo apt-get upgrade -y`

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

  


catkin build # if catkin not found, install catkin tools (apt-get install python-catkin-tools)

# NOTE: If the previous catkin build command gave an error related to the franka_ros_interface
# please try building with the -DFranka_DIR:PATH flag specified (attention replace path to libfranka/build below)
# catkin build -DFranka_DIR:PATH=/path/to/your/libfranka/build

  

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

Finally apply find /home/vm/hand_ws/src -type f -iname "*.py" -exec chmod +x {} \;
 to make all python scripts executable and 
 find /home/vm/hand_ws/src -type f -iname "*.cpp" -exec chmod +x {} \;


### Starting Procedure
The whole system gets started in the following order. Don't be too quick with executing the commands below and execute each of them in a seperate terminal.
1. Start the panda_simulator \
`roslaunch panda_gazebo panda.launch` 
2. Start the panda_hithand_moveit_config \
`roslaunch panda_hithand_moveit_config panda_hithand_moveit.launch` 
3. Start the grasp_pipeline. This exposes the the grasping servers. Currently this does not do anything in and of itself. But you can for example spawn objects in Gazebo \ 
`roslaunch grasp_pipeline grasp_pipeline_servers.launch` 