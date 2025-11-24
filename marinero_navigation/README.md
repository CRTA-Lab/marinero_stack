# MARINERO Navigation

This package provides navigation launch files and YAML configuration for autonomous navigation of the **MARINERO** robot in the Marina Punat environment.  
It supports both **MPPI** and **DWB** navigation algorithms for safe movement through allowed areas of the marina.

---

## Requirements

- **ROS 2 Humble** (or compatible ROS 2 distribution)  
- ROS 2 packages:  
  - `nav2_bringup`  
  - `nav2_map_server`

---

## Installation

Clone the repository into your ROS 2 workspace and build with all dependencies:

```
  cd workspace_folder/src
  git clone https://github.com/albic98/marinero_pointclouds.git
  cd ..
  colcon build --symlink-install
  source install/setup.bash
```
Ensure that other required MARINERO repositories (e.g., marinero_pointclouds, marinero_control) are also cloned and built in the same workspace for full functionality.

---

## Getting started

This package contains the costmap of Marina Punat where the robot is tested: 
- `marina_punat_with_docking_station.pgm` - the costmap image
- `marina_punat.yaml` - corresponding YAML file mapping the costmap to real-world coordinates.

Navigation configuration files (YAML) can be modified depending on the robot controller or navigation algorithm:
- `dwb_nav2_marinero_4wis4wid_drive_params.yaml`
- `dwb_nav2_marinero_skid_steer_params.yaml`
- `mppi_nav2_marinero_4wis4wid_drive_params.yaml` (default for current navigation)

The YAML file used for navigation is specified in `localization_navigation.launch.py` and can be updated as needed.

---

## Usage/Examples

---

#### Visualize the costmap:
```
    ros2 launch marinero_navigation marina_punat.launch.py
```

#### Launch the full navigation workflow:

```
    ros2 launch marinero_navigation localization_navigation.launch.py
```

---

## Connection to MARINERO ecosystem

This package works together with `marinero_simulations` and `marinero_control` to provide full simulation and autonomous navigation.

Launching `localization_navigation.launch.py` automatically starts the required navigation nodes and integrates with odometry, controllers, and pointcloud visualization.
