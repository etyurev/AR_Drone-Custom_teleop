Prerequisites
    Install OpenCV by this tutorial https://www.learnopencv.com/install-opencv3-on-ubuntu/ (OpenCV4 might also work, I dont know).

    sudo apt-get install ros-kinetic-image-view
    sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
    sudo apt-get install ros-kinetic-cv-bridge
    sudo apt-get install ros-kinetic-vision-opencv

Download the 'P4GUI' folder and place it next to the custom_keyboard folder in ~/ardrone_simulator/src

    cd ~/ardrone_simulator/
    catkin_make
    source devel/setup.bash
    
From here you will need 3 terminals, one for the simulation, one for the custom_teleop one for P4GUI

1a. Simulation

    roslaunch cvg_sim_gazebo ardrone_testworld.launch

1b. ar drone
    
    roslaunch ardrone_autonomy ardrone.launch
    
2. GUI opencv node

    rosrun p4GUI p4GUI_node

The p4Gui node subscribes to the /ardrone/front/image_raw topic and converts it into a opencv Mat c++ object. For the real drone the topic to subscribe to might differ.
