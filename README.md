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

Give the `ros2_control_node` executable special privileges to communicate over ethercat without sudo:

```
sudo apt install patchelf
cd /opt/ros/humble/lib/controller_manager
sudo setcap cap_net_raw=ep ./ros2_control_node
sudo patchelf --set-rpath /opt/ros/humble/lib/:/lib/x86_64-linux-gnu/ ros2_control_node
```

This only needs to be run once, or every time you install a new `controller_manager` debian.

Then you can launch the demo:

`ros2 launch synapticon_ros2_control single_dof.launch.py`

You should see a single-dof robot with torque control activated.
