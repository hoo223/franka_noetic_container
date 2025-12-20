sudo apt install -y \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-realsense2-camera \
    ros-noetic-combined-robot-hw \
    ros-noetic-boost-sml \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-moveit

CURRENT_DIR=$(pwd)
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone -b noetic-devel https://github.com/moveit/panda_moveit_config.git
git clone -b noetic-devel https://github.com/frankarobotics/franka_ros.git
git clone -b ros1-legacy https://github.com/realsenseai/realsense-ros.git
cd .. && catkin_make && source devel/setup.bash

cd "$CURRENT_DIR"