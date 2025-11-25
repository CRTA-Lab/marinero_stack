# MARINERO stack

A complete set of **ROS 2** packages providing the digital twin, controllers, localization pipeline, pointcloud processing, and navigation system for the **MARINERO** robot.
The stack includes full simulation capabilities in **Gazebo**, **RViz**, and **Mapviz**, supporting both `differential drive` and `4WIS4WID` control modes.

---

## Usage/Examples

#### Start the full Gazebo simulation (using the 4WIS4WID controller with `ros2_control.xacro`):
```
  ros2 launch marinero_simulations gazebo_simulation.launch.py
```

#### Launch the complete navigation workflow:

```
    ros2 launch marinero_navigation localization_navigation.launch.py
```

#### Start remote navigation via the mobile app:
```
    ros2 launch goals.launch.py
```

## Support

For more details, refer to the packages inside the `marinero_stack` repository.

Required **LIDAR data** and Gazebo `models` are available on CRTA SharePoint: https://fsbhr.sharepoint.com/:f:/t/crta/IgDjoSHoX-niRbSaTBpjfulSAVM3a0PhhsyRmNpBvphrFP0?e=yFOt1z

