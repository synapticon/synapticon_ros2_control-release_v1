## Description ##

A ros2_control driver for Synapticon Somanet motor drivers. Utilizes `soem`.

## Usage ##

`colcon build`

### Simple zero-torque control example on hardware ###

This is a simple executable, separate from ros2_control. Use it to check your ethercat connection.

`cd your_workspace/install/synapticon_ros2_control/bin`

Give the application the correct privileges. This only needs to be run once, or after every build:

`sudo setcap cap_net_raw=ep torque_control_executable`

`torque_control_executable`

The motor seeks to maintain zero torque.

### ros2_control example on actual hardware ###

#### Set up a systemd service to grant socket communication privileges to ros2_control_node ####

We run `ros2_control_node` in a systemd service because it needs elevated permissions to communicate over ethercat (without `sudo`).

Set it up like so:

`sudo touch /etc/systemd/system/ros2_control_node.service`

Add this to the service file and save it:

```
[Unit]
Description=Launch ros2_control_node with socket permissions

[Service]
Type=simple
User=apptronik
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash; source /home/apptronik/.bashrc; source /home/apptronik/workspaces/isaac_ros-dev/install/setup.bash'
# Write the user environment to file, for debugging
#ExecStartPre=/bin/bash -c 'env > /home/apptronik/Documents/ros_env_before_start.txt'

# This is essentially a copy of my normal user env
Environment="AMENT_PREFIX_PATH=/home/apptronik/workspaces/isaac_ros-dev/install/synapticon_ros2_control:/opt/ros/humble"
Environment="HOME=/home/apptronik"
Environment="LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib"
Environment="PATH=/opt/ros/humble/bin:/usr/lib/ccache:/home/apptronik/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin"
Environment="PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
Environment="ROS_DISTRO=humble"
Environment="ROS_DOMAIN_ID=42"
Environment="ROS_LOCALHOST_ONLY=1"
Environment="ROS_PYTHON_VERSION=3"
Environment="ROS_VERSION=2"
Environment="ROSCONSOLE_FORMAT=[${severity}] - ${node}: [${time}] ${message}"
Environment="USER=apptronik"
Environment="USERNAME=apptronik"

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

Alternatively, launch everything as sudo. This is not recommended but it prints more nicely, for debugging.

Open 2 terminals and do sourcing for each of them like so:

`sudo -i`

`source /home/your_user/.bashrc`

`source /your/workspace/install/setup.bash`

Then do the launching:

`ros2 launch synapticon_ros2_control elevated_permissions.launch.py`

`ros2 launch synapticon_ros2_control single_dof.launch.py`

Publish commmands from the command line like so:

`ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray data:\ [0.1]`
