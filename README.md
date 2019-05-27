# P4 project - Group 470 - Drone control

This github repository is a collection of the ROS nodes developed for this P4 project. The code is therefore meant to be executed as ros nodes.

Prerequisites:
    
    Royale SDK - for the pico flexx camera
    sudo apt-get install ros-kinetic-teleop-twist-keyboard
    ar drone autonomy ros package
    OpenCv
    

# AR_Drone-Custom_teleop

AR Drone simulation in Gazebo with Custom teleop keyboard


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



# OpenCV GUI (beyond visual line of flight)

See readme in P4GUI

# i_tongue_package

See setup in folder

# Drift package
Used to stabilize the drift of the drone and stabilize its hovering. Currently, it has only been tested in the simulation.

Add the package to a workspace, change line 163 in te cmakelist and build it.

Pretty much just run the code.
rosrun drift_pkg v1_drift.py

It will say ready, when it is ready. Otherwise there is a problem with subscribing to data from the bottom camera.
It will work if it is running along with the simulation of the drone.

Turning on and off with these commands in terminal(true for activation)
rostopic pub /hover_mode std_msgs/Bool '{data: True}'
rostopic pub /hover_mode std_msgs/Bool '{data: False}'
