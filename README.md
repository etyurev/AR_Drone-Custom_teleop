# AR_Drone-Custom_teleop

AR Drone simulation in Gazebo with Custom teleop keyboard

Prerequisites:

    sudo apt-get install ros-kinetic-teleop-twist-keyboard

How to install the simulator:

    Install gazebo7 and ardrone_autonomy package

    Create a workspace for the simulator

    mkdir -p ~/ardrone_simulator/src
    cd  ~/ardrone_simulator/src
    catkin_init_workspace

    Download package

    git clone https://github.com/iolyp/ardrone_simulator_gazebo7

    Build the simulator

    cd ..
    catkin_make

    Source the environment

    source devel/setup.bash
    
Creating the package for the custome keyboard:
    
    cd ~/ardrone_simulator/src
    catkin_create_pkg custom_keyboard
    cd custom_keyboard
    mkdir -p src
    cd src
    add to src file teleop_twist_keyboard.py from this repository
    chmod +x teleop_twist_keyboard.py
    
    cd ~/ardrone_simulator
    catkin_make
    source devel/setup.bash
    

How to run a simulation:

Open 3 terminals or Terminator
1 Terminal roscore
2 Terminal cd ~/ardrone_simulator
           source devel/setup.bash
           roslaunch cvg_sim_gazebo ardrone_testworld.launch
3 Terminal: Open when Gazebo starts simulation
           cd ~/ardrone_simulator
           source devel/setup.bash
           rosrun custom_teleop teleop_twist_keyboard.py
           1-take off; 2 land;
           

    Run a simulation by executing a launch file in cvg_sim_gazebo package:

    roslaunch cvg_sim_gazebo ardrone_testworld.launch

How to run a simulation using ar_track_alvar tags:

    Move the contents of ~/ardrone_simulator/src/cvg_sim_gazebo/meshes/ar_track_alvar_tags/ to ~/.gazebo/models

    Run simulation

    roslaunch cvg_sim_gazebo ar_tag.launch



# OpenCV GUI

Prerequisites
    Install OpenCV by this tutorial https://www.learnopencv.com/install-opencv3-on-ubuntu/ (OpenCV4 might also work, I dont know).

    sudo apt-get install ros-indigo-image-view
    sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
    sudo apt-get install ros-kineticcv-bridge
    sudo apt-get install ros-kinetic-vision-opencv

Download the 'P4GUI' folder and place it next to the custom_keyboard folder in ~/ardrone_simulator/src

    cd ~/ardrone_simulator/
    catkin_make
    source devel/setup.bash
    
From here you will need 3 terminals, one for the simulation, one for the custom_teleop one for P4GUI

1. Simulation

    roslaunch cvg_sim_gazebo ardrone_testworld.launch
    
2. Custom teleop

    rosrun custom_keyboard teleop_twist_keyboard.py 
3. 
    rosrun p4GUI p4GUI_node

The p4Gui node subscribes to the /ardrone/front/image_raw topic and converts it into a opencv Mat c++ object. For the real drone the topic to subscribe to might differ.
