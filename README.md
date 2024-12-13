## Description ##

A ros2_control driver for Synapticon Somanet motor drivers. Utilizes `soem`.

## Usage ##

`colcon build`

`cd your_workspace/isaac_ros_dev/install/synapticon_ros2_control/bin`

`sudo setcap cap_net_raw=ep torque_control_executable`

TODO: this doesn't work yet

`ros2 run synapticon_ros2_control torque_control_executable --prefix ethercat_grant`

TODO: remove ethercat_grant if it continues being useless
