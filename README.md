  

  

# Panda Simulator

  

  

A **Gazebo simulator** for the Franka Emika Panda robot with ROS interface with the option to attach a DLR-HIT Hand II as end-effector, providing exposed **controllers** and real-time **robot state feedback** similar to the real robot when using the *franka-ros* package.

  
## Installation Process

This section guides through the whole installation process, not only for the panda_hithand_simulator but for the full grasping system. This includes 

- Gazebo 9 from source with DART support.
- CUDA 10.1.
- ROS melodic.
- libfranka and franka-ros
- wide range of ros-melodic and python packages

#### **Install Gazebo 9 from source**

- The first step is to install Gazebo 9 from source. Installing from source is necessary to get full DART support, which is most suitable for grasping applications under all solvers available for Gazebo. Ideally you would have a fresh Ubuntu install or at least make sure you have no Gazebo/ ROS installed on your system as I found this to interfere with the installation process.

- __IMPORTANT NOTES for the next step__: 
    1. Use the recommended `/home/$USER/local` as installation path
    2. After cloning the Gazebo repository be sure to `git checkout gazebo9` as building the main branch will not work.

- Please follow the steps outlined here CAREFULLY: [Gazebo Installation from Source](http://gazebosim.org/tutorials?tut=install_from_source&cat=install)


#### **Install CUDA 10.1**

- In order to run the ML-based grasping inference process you will need pytorch 1.7.1 which needs CUDA 10.1

- Follow the steps closely in the linked installation script and restart your computer after installation. I would suggest first removing any NVIDIA / CUDA software from your system as I found this to interfere with the installation process.

- __NOTE__: for the next step: 
    Instead of executing wget https://developer.nvidia.com/compute/machine-learning/cudnn/secure/7.6.5.32/Production/10.1_20191031/cudnn-10.1-linux-x64-v7.6.5.32.tgz you will need to find the that file cudnn-10.1-linux-x64-v7.6.5.32.tgz on NVIDIAs website, after logging in. Running this command will fail, because you need to authorise with NVIDIA first.


- [CUDA 10.1 Installation] Please refer to links below:  
https://developer.nvidia.com/cuda-toolkit-archive  
https://docs.nvidia.com/cuda/archive/10.1/cuda-installation-guide-linux/index.html#runfile  

- Make sure you have restarted your computer. Afterwards verify the cuda installation via nvidia-smi and nvcc -V



#### **Install ROS melodic**

1. ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
2. ```bash
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    ```
3. ```bash
    sudo apt-get update
    ```
4. ```bash
    sudo apt-get install ros-melodic-desktop
    ```
5. ```bash
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
6. ```bash
    sudo apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo rosdep init
    rosdep update
    ```

#### **Installing Libfranka and FrankaROS**

  

  

1. Install and build libfranka v0.6.0 from source

  

  

- Remove any existing libfranka

  

    ```bash
    sudo apt remove "*libfranka*"
    ```

  

- Install dependencies

  

    ```bash
    sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
    ```

  

- Clone the source code

  

    ```bash
    git clone --recursive https://github.com/frankaemika/libfranka
    cd libfranka
    ```

  

- Git checkout version 0.6.0

  

    ```bash
    git checkout 2b99ab9
    git submodule update
    ```

  

- In the source directory, create a build directory and run CMake

  

    ```bash
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build .
    ```

  

2. Install and build Franka-ROS v0.6.0

  

- Create a catkin workspace

  

    ```bash
    cd /path/to/desired/folder
    mkdir -p hithand_ws/src
    cd hithand_ws
    source /opt/ros/melodic/setup.sh
    catkin_init_workspace src
    ```
  

- Clone franka_ros from Github and checkout correct version

  

    ```bash
    git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
    cd src/franka_ros/
    git checkout 49e5ac1

    cd ../../  
    ```
- Install missing dependencies

  

    ```bash
    rosdep install --from-paths src --ignore-src --rosdistro melodic -y --skip-keys libfranka
    ```

    - In case of error `dkpg: error processing archive /var/cache/apt/archives/libomp5-7... (--unpack)`
        ```bash
        sudo apt-get -o Dpkg::Options::="--force-overwrite" install libomp5-7
        ```

- Install franka-ros, replace the /path/to/libfranka/build with your libfranka/build directory
    ```bash
    catkin build franka_ros -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
    source devel/setup.sh
    ```

    - In case of `error:  catkin command not found`, run: 
        ```bash
        sudo apt-get install python-catkin-tool
        ```

  

#### **Other dependencies**
1. 
    ```bash
    sudo apt-get update && sudo apt-get install -q -y build-essential git swig sudo python-future libcppunit-dev python-pip
    ```

2.
    ```bash
    sudo apt-get update && sudo apt-get install -y python-catkin-tools ros-melodic-gazebo-ros-control ros-melodic-rospy-message-converter ros-melodic-effort-controllers ros-melodic-position-controllers ros-melodic-joint-state-controller python-pip ros-melodic-moveit ros-melodic-moveit-commander ros-melodic-moveit-visual-tools ros-melodic-tf2-sensor-msgs ros-melodic-rosbridge-server ros-melodic-tf2-web-republisher ros-melodic-ros-control ros-melodic-moveit ros-melodic-trac-ik-kinematics-plugin ros-melodic-ros-numpy ros-melodic-trac-ik-python ros-melodic-image-proc ros-melodic-image-pipeline
    python3-pip
    ```

3.
    ```bash
    pip install --upgrade pip
    ```

4.
    ```bash
    sudo apt-get update && sudo apt-get upgrade -y
    ```
  
<!-- #### **SKIP THIS: [Clone the Hithand Grasping Group**]
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
    ``` -->
#### **More Dependencies**

<!-- 1. Update dependency packages. This will install `franka-panda-description`, `franka-ros-interface`, `orocos-kinematic-dynamics` and `gazebo-realsense-plugin` from the Gitlab Hithand group:

    ```bash
    wstool init
    wstool merge panda-simulator/dependencies.rosinstall # If this fails, just install all repositories in dependencies.rosinstal manually.
    wstool up

    cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc

    cd ../.. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

    ``` -->
1. Install orocos-kinematics

    ```bash
    cd hithand_ws/src

    git clone https://github.com/qianbot/orocos-kinematics-dynamics.git

    cd orocos-kinematics-dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc

    cd ../..

    catkin build orocos_kinematics_dynamics
 
   ```
2. franka-ros-interface
    ```bash
    cd hithand_ws/src

    git clone https://github.com/qianbot/franka-ros-interface.git

    cd franka-ros-interface

    git checkout v0.6.0

    cd ../..

    catkin build franka_ros_interface -DFranka_DIR:PATH=/path/to/libfranka/build
    ```

3. franka-panda-description
    ```bash
    cd hithand_ws/src

    git clone https://github.com/qianbot/franka-panda-description.git

    cd ..

    catkin build franka_panda_description -DFranka_DIR:PATH=/path/to/libfranka/build
    ```

4. Hithand ros
    ```bash
    cd hithand_ws/src

    git clone https://github.com/qianbot/hithand_ros.git

    cd ..

    catkin build hithand_control hithand_gazebo hithand_description
    ```
5. Robotiq ros
    ```bash

    sudo apt-get install -y ros-melodic-socketcan-interface ros-melodic-soem

    cd hithand_ws/src

    git clone https://github.com/davidmartinez13/robotiq
    cd ..

    catkin build robotiq
    ```
6. Robotiq ros control pkg:
    ```bash
    cd hithand_ws/src

    git clone https://github.com/davidmartinez13/robotiq_3f_ros_pkg.git

    cd ..

    catkin build robotiq_3f_ros_pkg
    ```
7. Trajectory smoothing
    ```bash
    cd hithand_ws/src

    git clone https://github.com/qianbot/trajectory-smoothing.git

    cd ..

    catkin build trajectory_smoothing
    ```

8. Gazebo realsense plugin
    ```baseh
    cd hithand_ws/src

    git clone https://github.com/qianbot/realsense-gazebo-plugin
    cd ..

    catkin build realsense_gazebo_plugin
    ```
  
#### **Building the Package itself**


Once the dependencies are met, the package can be installed using catkin build (preferred over catkin_make):
    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash

    cd hithand_ws/src

    git clone https://github.com/qianbot/panda-simulator.git

    cd ..

    catkin build panda_simulator -DFranka_DIR:PATH=/path/to/your/libfranka/build 
    # if catkin not found, install catkin tools (apt-get install python-catkin-tools)

    source devel/setup.bash

## Usage

- The simulator can be started by running:

    ```bash
    roslaunch panda_gazebo panda_hithand.launch
    ```

- Now you test the control by sending a command to the corresponding controller topic. E.g.

    ```bash
    rostopic pub /panda_hithand/panda_j1_position_controller/command std_msgs/Float64 "data: 0.0"
    ```
- To spawn the robotiq dummy gripper in gazebo run: 

    ```bash
    roslaunch robotiq_3f_gripper_articulated_gazebo robotiq_gripper_empty_world.launch
    ```
- To spawn the robotiq gripper in gazebo run: 

    ```bash
    roslaunch robotiq_3f_gripper_articulated_gazebo robotiq_gripper_empty_world_macro.launch
    ```

    - Test the control with the provided python script:
        ```bash
        cd /robotiq/robotiq_3f_gripper_control/nodes/

        python2 Robotiq3FGripperSimpleController.py
        ```
        - The following commands are available from the python script:
    
            r: Reset

            a: Activate

            c: Close

            o: Open

            b: Basic mode

            p: Pinch mode

            w: Wide mode

            s: Scissor mode

            (0-255): Go to that position

            f: Faster

            l: Slower

            i: Increase force

            d: Decrease force

    - Or, control the gripper with ros services:
        
        - If in sim, then launch the listener for sim:
            ```
            roslaunch robotiq_3f_driver listener_sim.launch
            ```
        - If not on sim, launch the listener:
            ```
            roslaunch robotiq_3f_driver listener.launch ip_address:=192.168.1.11
            ```
            Both ways the ROS services and gripper will be activated.\
            Try sending some commands as:

            ```
            rosservice call /robotiq_3f_gripper/activate
            rosservice call /robotiq_3f_gripper/set_mode wide
            rosservice call /robotiq_3f_gripper/set_position 200
            rosservice call /robotiq_3f_gripper/set_position 150
            rosservice call /robotiq_3f_gripper/set_position 0
            ```
        


## Grasping Pipeline
For the entire grasping pipeline you will need more packages

### Installation
1. Panda Hithand Moveit Config\
    Clone the panda-hithand-moveit package also in the /src folder \
    ``` bash
    cd hithand_ws/src

    git clone https://github.com/qianbot/panda-hithand-moveit-config.git

    cd ..

    catkin build panda_hithand_moveit_config
    ``` 

2. Grasp pipeline\
    Clone the grasp-pipeline package which provides the core grasping client-server functionality. \
    ```bash
    cd hithand_ws/src 

    git clone https://github.com/qianbot/grasp-pipeline.git

    cd ..

    catkin build grasp_pipeline
    
    pip install -r grasp-pipeline/misc/requirements.txt
    ```

3. Bashrc Modifications\
    For the whole system to work some modifications have to made to your .bashrc file. (This is analogous for other shells).
    Replace /path/to/your/hithand_ws with the path to your catkin workspace.
    ```bash
    echo "source /path/to/your/hithand_ws/devel/setup.bash" >> ~/.bashrc
    ``` 

4. Make executabe\
    ```bash
    find /path/to/your/hithand_ws/src -type f -iname "*.py" -exec chmod +x {} \;

    find /path/to/your/hithand_ws/src -type f -iname "*.cpp" -exec chmod +x {} \;
    ```


### Starting Procedure
The whole system gets started in the following order. Don't be too quick with executing the commands below and execute each of them in a seperate terminal.
1. Start the panda_simulator \
`roslaunch panda_gazebo panda_hithand.launch` 
    - or to spawn robotiq next to panda:\
    `roslaunch panda_gazebo panda_robotiq3f.launch`\
     Test the control of panda with mounted robotiq by sending a command to the corresponding controller topic. E.g.

        ```bash
        rostopic pub /panda/panda_j7_position_controller/command std_msgs/Float64 "data: 1.0"
        ```
        Also, you can test the gripper functionality with the Robotiq3FGripperSimpleController.py or via ROS services mentioned in the Usage section.

2. Start the panda_hithand_moveit_config \
`roslaunch panda_hithand_moveit_config panda_hithand_moveit.launch` 
    - or to spawn robotiq3f moveit with panda:\
        `roslaunch robotiq_3f_rviz_moveit_panda panda_robotiq3f_moveit.launch`
3. Start the grasp_pipeline. This exposes the the grasping servers. Currently this does not do anything in and of itself. But you can for example spawn objects in Gazebo \
`roslaunch grasp_pipeline grasp_pipeline_servers.launch` 
    - or to Start the grasp_pipeline for robotiq3f:\
        `roslaunch grasp_pipeline grasp_pipeline_servers_robotiq.launch`
