## Project application
This repository contains custom ROS2 packages developed for use with Gazebo simulations for navigation over a TCP/IP connection. The packages were tested with the ROS2 Humble distribution.
Use any mobile app that can create a client which can then connect to the simulation. Used application: TCP UDP Server & Client from "Play Store".


## Getting started
Clone the repository and build the workspace:
```
    cd workspace_folder/src
   git clone https://github.com/albic98/goal_assignment.git
   cd ..
   source /opt/ros/<distro>/setup.bash
   colcon build --symlink-install
   source install/setup.bash
```

## Usage
To run the system, all three nodes must be launched in separate terminal windows using the following commands:
```
    ros2 run tcp_client_package tcp_client_node
    ros2 run location_package goal_executor
    ros2 run location_package coordinate_logger
```

   Or run launch file: 
```
    ros2 launch goals.launch.py
```

Goal coordinates can be selected directly from the Mapviz GPS map using the mouse pointer. Use the local coordinates displayed in the `map` frame as the reference.

Message examples:
```
    startFile
    (108.43,372.49,1.5)
    (-46.11,724.8,2.0)
    (22.89,814.1,0.5)
    .
    .
    .
    home
    endFile
```

Any number of coordinates (goals) can be entered. For aborting the mission simply enter: `abortMission`.

Each time the robot arrives at the coordinates it sends a picture on the link: `http://{self.listen_host}:{port}/latest.png")` where `port` is your acssigned port and slef.`listen_host` is the IP address on which your are connected.

## Important 
Before building the workspace, make sure to update the IP address and port in the /tcp_client_package/tcp_client_package/tcp_client_node.py to match your system setup.