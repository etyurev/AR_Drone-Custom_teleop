# P4 project - Group 470 - Drone control

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
