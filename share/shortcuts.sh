alias eb='micro ~/.bashrc'
alias sb='source ~/.bashrc'
alias cm='catkin_make'
alias sd='source devel/setup.bash'
alias cw='cd ~/share/catkin_ws'
alias rscam='roslaunch realsense2_camera rs_camera.launch align_depth:=true'
alias home='roslaunch franka_example_controllers move_to_start.launch'
alias cartimp='roslaunch franka_example_controllers cartesian_impedance_example_controller.launch'
alias smnode='roslaunch spacenav_node classic.launch'
alias fvis='roslaunch franka_interface franka_visualization.launch'
alias fmain='roslaunch franka_interface franka_main.launch'
alias grip='roslaunch franka_gripper franka_gripper.launch robot_ip:=172.16.0.2'
alias grasp='roslaunch franka_interface move_to_grasp.launch'
alias reach='rosrun franka_interface approach_phase_node.py '
alias demo='roslaunch demo_traj demo_trajectory.launch'
alias teleop='roslaunch franka_interface teleop.launch'

export DISPLAY=:0
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/openrobots/lib
export ROBOT_IP=172.16.0.2
export ROS_MASTER_URI=http://10.107.1.30:11311
export ROS_HOSTNAME=10.107.1.30
export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash

spacenavd 
cw && sd