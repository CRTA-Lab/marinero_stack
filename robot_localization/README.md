# Robot_localization

This package provides **launch files** and **YAML configuration files**, and **C++ scripts** for localizing the **MARINERO** robot. 

---

## Installation

Clone the repository and build the workspace:
```
    cd <workspace_folder>/src
    git clone https://github.com/albic98/robot_localization.git
    cd ..
    source /opt/ros/<distro>/setup.bash
    colcon build --symlink-install
    source install/setup.bash
```

**Note**: Ensure that other required MARINERO repositories (e.g., `marinero_simulations`, `marinero_control`) are also cloned and built in the same workspace for full functionality.

---

## Getting started

This package contains multiple launch files, each with a corresponding YAML configuration file and supporting C++ scripts.

### Launch files

- `dual_ekf_navsat_example.launch.py` - Example launch file combining EKF and Navsat localization using IMU, GPS, odometry, and velocity (`cmd_vel`) data

- `ekf.launch.py` - Launch file for EKF (Extended Kalman Filter) localization

- `ukf.launch.py` - Launch file for UKF (Unscented Kalman Filter) localization

- `navsat_transform.launch.py` - Launch file for Navsat localization using a GPS sensor plugin

- `merged_ekf_navsat.launch.py` - Launch file that integrates EKF and Navsat localization from `ekf.launch.py` and `navsat_transform.launch.py`, using IMU, GPS, odometry, and velocities (`cmd_vel`), adjusted specifically for the MARINERO robot

YAML configuration files have the same names as their corresponding launch files.

---

## Usage

The currently used launch file is:
```
    ros2 launch robot_localization merged_ekf_navsat.launch.py
```

This setup provides **odometry**, **velocity** (`cmd_vel`), **GPS**, and **IMU sensor-based localization**.

- It can be launched manually or integrated into the localization_navigation.launch.py file in the marinero_navigation
package.

- Currently, it is `not integrated by default` to save computing power.

---

## Additional resources

For more details on robot localization in ROS 2, see the official `robot_localization` repository:
https://github.com/cra-ros-pkg/robot_localization.