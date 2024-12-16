## Description ##

A ros2_control driver for Synapticon Somanet motor drivers. Utilizes `soem`.

## Usage ##

`colcon build`

### Simple zero-torque control executable ###

This is a simple executable, separate from ros2_control. Use it to check your ethercat connection.

`cd your_workspace/install/synapticon_ros2_control/bin`

Give the application the correct privileges. This only needs to be run once, or after every build:

`sudo setcap cap_net_raw=ep torque_control_executable`

`torque_control_executable`

The motor seeks to maintain zero torque.

### ros2_control ###

Launch a simple demo:

`ros2 launch synapticon_ros2_control single_dof.launch.py`

The controller itself should be launched as `sudo` for ethercat access:

TODO - nodes launched as sudo can't communicate with other nodes

`sudo -i`

`source /home/your_user/.bashrc`

`source your_workspace/install/setup.bash`

`ros2 launch synapticon_ros2_control elevated_privileges.launch.py`
