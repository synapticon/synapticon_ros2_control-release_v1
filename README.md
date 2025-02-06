## Description ##

A ros2_control driver for Synapticon Somanet Node motor drivers. Utilizes `soem`.

## Requirements ##

This package is intended for Linux OS, Somanet Node motor drivers, and ROS2 Humble as of January 2025.

## Initial Synapticon setup ##

Go through the initial setup of your Synapticon servo drive with a Synapticon rep. This will involve setting gains, number of poles, etc. I used the OBLAC Servo Drive Commissioning page.

## Usage ##

Create a ROS2 workspace:

`mkdir -p ~/ros2_ws/src`

`cd ~/ros2_ws/src`

Clone this repository into the `src` folder then build it:

`cd ~/ros2_ws`

`rosdep install --from-paths src -y --ignore-src`

`colcon build`

### Ethernet connection ###

I configured a wired ethernet connection with the following settings:

- MTU: automatic

- IPv4: Manual, address 192.168.0.0, netmask 255.255.255.0, gateway 192.168.0.1, DNS automatic

- IPv6: automatic

Connect your PC to the Synapticon drive with an ethernet cable.

#### Changes needed depending on your ethernet interface name ####

If your ethernet interface name is not `eno0` you need to make these changes:

- `src/torque_control_executable.cpp`, rename `eno0`

- `description/ros2_control/single_dof.ros2_control.xacro`, rename `eno0`

- recompile with `colcon build`

### Simple zero-torque control example on hardware ###

This is a simple executable, separate from ros2_control. The motor seeks to maintain zero torque. Use it to check your ethercat connection.

`cd ros2_ws/install/synapticon_ros2_control/bin`

`sudo ./torque_control_executable`

### ros2_control example on actual hardware ###

#### Option 1: Set up a systemd service to grant socket communication privileges to ros2_control_node ####

We often run `ros2_control_node` in a systemd service because it needs elevated permissions to communicate over ethercat (without `sudo`).

Set it up like so:

`sudo touch /etc/systemd/system/ros2_control_node.service`

Add this to the service file and save it:

```
[Unit]
Description=Launch ros2_control_node with socket permissions

[Service]
Type=simple
User=your_user
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash; source /home/your_user/.bashrc; source /home/your_user/workspaces/isaac_ros-dev/install/setup.bash'
# Write the user environment to file, for debugging
#ExecStartPre=/bin/bash -c 'env > /home/your_user/Documents/ros_env_before_start.txt'

# This is essentially a copy of my normal user env
Environment="AMENT_PREFIX_PATH=/home/your_user/workspaces/isaac_ros-dev/install/synapticon_ros2_control:/opt/ros/humble"
Environment="HOME=/home/your_user"
Environment="LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib"
Environment="PATH=/opt/ros/humble/bin:/usr/lib/ccache:/home/your_user/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin"
Environment="PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
Environment="ROS_DISTRO=humble"
Environment="ROS_DOMAIN_ID=42"
Environment="ROS_LOCALHOST_ONLY=1"
Environment="ROS_PYTHON_VERSION=3"
Environment="ROS_VERSION=2"
Environment="ROSCONSOLE_FORMAT=[${severity}] - ${node}: [${time}] ${message}"
Environment="USER=your_user"
Environment="USERNAME=your_user"

ExecStart=/opt/ros/humble/bin/ros2 launch synapticon_ros2_control elevated_permissions.launch.py
AmbientCapabilities=CAP_NET_RAW

[Install]
WantedBy=multi-user.target
```

`sudo systemctl daemon-reload`

Start the service:

`sudo systemctl restart ros2_control_node.service`

Check service status and see the ROS console logging:

`sudo systemctl status ros2_control_node.service`

#### Launching the ros2_control example ####

`sudo systemctl restart ros2_control_node.service`

`ros2 launch synapticon_ros2_control single_dof.launch.py`

Stop the `ros2_control_node` with:

`sudo systemctl stop ros2_control_node.service`

#### Option 2: Launch everything as sudo ####

As an alternative to systemd, launch everything as sudo. This is not recommended in the long term but it's easy and it prints more nicely, for debugging.

Open 2 terminals and do sourcing for each of them like so:

`sudo -i`

`source /home/your_user/.bashrc`

`source /your/workspace/install/setup.bash`

Then do the launching:

`ros2 launch synapticon_ros2_control elevated_permissions.launch.py`

`ros2 launch synapticon_ros2_control single_dof.launch.py`

Check the status like so:

`ros2 control list_controllers`

Publish commmands from the command line like so:

`ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController activate_controllers:\ ["forward_velocity_controller"]`

`ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray data:\ [0.1]`

"Quick Stop mode" is activated like so:

`ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['quick_stop_controller'], deactivate_controllers: ['forward_velocity_controller']}"`
