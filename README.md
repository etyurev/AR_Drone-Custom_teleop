# P4 project - Group 470 - Drone control

Place the following ros packages into the /src folder in a catkin workspace

    - AR_drone-Custom_teleop
    - P4GUI
    - depthDetection
    - i_tongue_pkg_f
    - obstacle_avoidance
    
The pico_flexx_python folder contains a ros publisher meant to be run on the micro-controller (Raspberry Pi)

This github repository is a collection of the ROS nodes developed for this P4 project. The code is therefore meant to be executed as ros nodes.

Prerequisites:
    
    Royale SDK - for the pico flexx camera
    sudo apt-get install ros-kinetic-teleop-twist-keyboard
    ar drone autonomy ros package
    OpenCv
    

# AR_Drone-Custom_teleop

AR Drone simulation in Gazebo with Custom teleop keyboard
See readme in custom_teleop_keyboard folder


# OpenCV GUI (beyond visual line of flight)

See readme in P4GUI

# depthDetection (human detection)

see readme in folder.

# i_tongue_package

For controlling the drone with the ITCI. Remember to connect via bluetooth.

See setup in folder.

# obstacle_avoidance

See setup in folder.

# pico_flexx_python

Place in the pico flexx sdk python samples folder and run with python3 on the Raspberry Pi 3b.
