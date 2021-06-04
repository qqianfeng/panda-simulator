  

  

# Panda Simulator

  

  

A **Gazebo simulator** for the Franka Emika Panda robot with ROS interface with the option to attach a DLR-HIT Hand II as end-effector, providing exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the *franka-ros* package.

  

  

## Features

  

  

- Low-level *controllers* (joint position, velocity, torque) available that can be controlled through ROS topics (including position control for gripper) or Python API.

  

  

- Real-time *robot state* (end-effector state, joint state, controller state, etc.) available through ROS topics.

  

  

- The *Franka ROS Interface* package (which is a ROS interface for controlling the real Panda robot) and PandaRobot Python API can also be used with the panda_simulator, providing kinematics and dynamics computation for the robot, and direct *sim-to-real* code transfer.

  

  

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

__IMPORTANT NOTES for the next step__: 
1. Use the recommended `/home/$USER/local` as installation path
2. After cloning the Gazebo repository be sure to `git checkout gazebo9` as building the main branch will not work.

Please follow the steps outlined here CAREFULLY: [Gazebo Installation from Source](http://gazebosim.org/tutorials?tut=install_from_source&cat=install)




#### Install CUDA 10.1

In order to run the ML-based grasping inference process you will need pytorch 1.4.0 which needs CUDA 10.1

Follow the steps closely in the linked installation script and restart your computer after installation. I would suggest first removing any NVIDIA / CUDA software from your system as I found this to interfere with the installation process.

__NOTE__: for the next step: 
- Instead of executing wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/7.6.5.32/Production/10.1_20191031/cudnn-10.1-linux-x64-v7.6.5.32.tgz you will need to find the that file cudnn-10.1-linux-x64-v7.6.5.32.tgz on NVIDIAs website, after logging in. Running this command will fail, because you need to authorise with NVIDIA first.


[CUDA 10.1 Installation](https://gist.github.com/vincentmaye/090159c493adbf6b3d8f39329c78d12c)

Make sure you have restarted your computer. Afterwards verify the cuda installation via nvidia-smi and nvcc -V



#### Install ROS melodic

Now follow the instruction from this link. Be sure to choose ros-melodic-desktop and **NOT** ros-melodic-desktop-full:
[ROS melodic Installation](http://wiki.ros.org/melodic/Installation/Ubuntu)


#### Installing Franka Emika Software

  
  

Due to recent updates this step became really easy and works with the binaries of libfranka and franka-ros.

Simply run:
```bash
 sudo apt-get install ros-melodic-libfranka ros-melodic-franka-ros
```

  

#### Other dependencies

```bash
sudo apt-get update && sudo apt-get install -q -y build-essential git swig sudo python-future libcppunit-dev python-pip
```


```bash
sudo apt-get update && sudo apt-get install -y python-catkin-tools ros-melodic-gazebo-ros-control ros-melodic-rospy-message-converter ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-state-controller python-pip ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools ros-melodic-tf2-sensor-msgs ros-melodic-rosbridge-server ros-melodic-tf2-web-republisher ros-melodic-ros-control ros-melodic-moveit ros-melodic-trac-ik-kinematics-plugin ros-melodic-ros-numpy ros-melodic-trac-ik-python ros-melodic-image-proc ros-melodic-image-pipeline
```

```bash
pip install --upgrade pip
```


```bash
sudo apt-get update && sudo apt-get upgrade -y
```

cd to panda-simulator repo:

```bash
pip install -r requirements.txt
```

  
  
#### Clone the Hithand Grasping Group
1. Install gitlabber
    ```bash
    pip3 install gitlabber
    ```
2. Get a personal access token from Gitlab
    - Go to your avatar in the top right corner.
    - Select **Edit profile**.
    - In the left sidebar, select **Access Tokens**.
    - Enter a name for the token.
    - Select all scopes.
    - Select **Create personal access token**
    - Copy the hash of the token.
3. Set the token as an environment variable: 
    ```bash
    export GITLAB_TOKEN=yourTokenHash
    ```
4. Set gitlab URL as environment variable:
    ```bash
    export GITLAB_URL=http://git.ar.int/
    ```
5. Create two folders at root. First 
    ```bash
    cd ~ && mkdir -p hand_ws/src && cd hand_ws
    ```
6. Copy all repositories from the "Hithand Grasp" group 
    ```bash
    gitlabber -i '/Deep Learning Group/Hithand Grasp**' ~/hand_ws/src
    ```
7. Move all repositories to src
    ```bash
    mv * ~/hand_ws/src && rm -rf Deep\ Learning\ Group/
    ```
#### Building the Package

  

1. Clone the repo:

  

```bash
cd hithand_ws/src
git clone git@git.ar.int:deeplearn/hithand-grasp/panda-simulator.git
```

  

2. Update dependency packages. This will install `franka-panda-description`, `franka-ros-interface`, `orocos-kinematic-dynamics` and `gazebo-realsense-plugin` from the Gitlab Hithand group:

  

  

```bash

  

wstool init

  

wstool merge panda-simulator/dependencies.rosinstall # If this fails, just install all repositories in dependencies.rosinstal manually.

  
  

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
1. Panda Hithand Moveit Config
- Clone the panda-hithand-moveit package also in the /src folder \
`git clone git@git.ar.int:deeplearn/hithand-grasp/panda-hithand-moveit-config.git` 

2. Grasp pipeline
- Clone the grasp-pipeline package which provides the core grasping client-server functionality. \
`git clone git@git.ar.int:deeplearn/hithand-grasp/grasp-pipeline.git`
- This is actually a bit more effort than the previous packages. First install the python requirements \
`cd grasp-pipeline`
`pip install -r requirements.txt`

3. Bashrc Modifications
- For the whole system to work some modifications have to made to your .bashrc file. (This is analogous for other shells). \
`vim ~/.bashrc` 

4. Make executabe 
- Apply  `find /home/vm/hand_ws/src -type f -iname "*.py" -exec chmod +x {} \;`
- and 
 `find /home/vm/hand_ws/src -type f -iname "*.cpp" -exec chmod +x {} \;`


### Starting Procedure
The whole system gets started in the following order. Don't be too quick with executing the commands below and execute each of them in a seperate terminal.
1. Start the panda_simulator \
`roslaunch panda_gazebo panda_hithand.launch` 

2. Start the panda_hithand_moveit_config \
`roslaunch panda_hithand_moveit_config panda_hithand_moveit.launch` 

3. Start the grasp_pipeline. This exposes the the grasping servers. Currently this does not do anything in and of itself. But you can for example spawn objects in Gazebo \
`roslaunch grasp_pipeline grasp_pipeline_servers.launch` 