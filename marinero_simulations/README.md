# MARINERO

Digital twin description files, controllers, and localization logic for the **MARINERO** robot in **ROS 2**.  
This package enables simulation of the robot in Gazebo and RViz, including both `differential drive` and `4WIS4WID` control modes.

---

Features

- Full MARINERO robot URDF/Xacro description

- 4WIS4WID steering & wheel control integration

- Differential drive (fallback mode)

- Gazebo & RViz simulation environment

- Marina world model with segmented zone loading (A, B, C)

- Pointcloud integration (via marinero_pointclouds)

- Utility scripts for map alignment, SDF conversion, and pier visualization

---

## Requirements  

- **ROS 2 Humble** (or compatible ROS 2 distribution)  
- Packages:  
  - `ros2_control`  
  - `controller_manager`  
  - `twist_mux`  
  - `joy`  
  - `teleop_twist_joy`
  - `gazebo`  

---

## Installation

Clone the repository in your ROS2 workspace.

```
  cd <workspace_folder>/src
  git clone https://github.com/albic98/marinero_simulations.git
```

After cloning build the package in your `<workspace_folder>/src`. For example `marinero_ws/src`.

```
  source /opt/ros/<distro>/setup.bash
  colcon build --symlink-install
  source install/setup.bash
```

For the simulation to work, clone and build the following repositories in the same `<workspace_folder>/src`:
```
  git clone https://github.com/albic98/marinero_control.git
  git clone https://github.com/albic98/marinero_pointclouds.git
```

Then build your workspace:
```
  source /opt/ros/<distro>/setup.bash
  colcon build --symlink-install
  source install/setup.bash
```

---

## Usage/Examples

#### Start the robot state publisher for MARINERO:

```
  ros2 launch marinero_simulations rsp.launch.py
```

#### Initiate the entire simulation using this command (4WIS4WID controller) with `ros2_control.xacro`:
```
  ros2 launch marinero_simulations gazebo_simulation.launch.py
```

#### Launch command for controllers if they do not start automatically:
```
  ros2 launch marinero_simulations controllers.launch.py
```

#### Initiate the entire simulation using this command (2 differential drive controller) with `gazebo_control.xacro`:
```
  ros2 launch marinero_simulations gazebo_simulation.launch.py use_ros2_control:=false use_4wis4wid:=false
```
---

## Script overview

`marina_gazebo_publisher.py` - Publishes the full marina SDF model into Gazebo.

`marina_sdf2marker.py` - Publishes the full pier model, zones A/B/C, into RViz.

`parse_sdf2marker.py` - Converts SDF models into Xacro files (not used in current simulation)

`pgm_image_rotator.py` - One-time tool used only when the origin or rotation of a new PGM map image is unknown. Helps align the costmap with the GPS/map/Gazebo world. Not used during normal operation.

`pose_z_offset.py` - Adjusted Z-offsets for goals/initial positions (used in earlier versions)

`segmented_gazebo_publisher.py` - Publishes only the current marina zone into Gazebo (improves performance)

`segmented_sdf2marker.py` - Publishes only the current pier segment in RViz

---

## Gazebo models

The `models` directory contains the Gazebo models used in the simulation.

**Note**: the project was developed against an older Gazebo simulator version — the models and launch files target that simulator. They should work, but you may encounter compatibility issues on newer Gazebo releases. If you run into problems, try using the simulator version used during development or contact the address in the Support section for help or access to alternate model files.

Due to file size limitations, some of the larger models could not be uploaded to the repository.
If you need access to the full set of models, please contact the email provided in the `Support` section.

---

## Support

For support, go to https://github.com/CRTA-Lab/marinero_stack.

For additional details, refer to the READMEs in:

- `marinero_control`
- `marinero_pointclouds`