Mapviz
======
| Humble | Iron | Rolling |
| :--- | :--- | :--- |
| [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mapviz__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mapviz__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Ibin_uJ64__mapviz__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__mapviz__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mapviz__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mapviz__ubuntu_jammy_amd64__binary/)

Mapviz is a [ROS](http://www.ros.org/) based visualization tool with a plug-in system similar to [RVIZ](http://wiki.ros.org/rviz) focused on visualizing 2D data.

![](https://github.com/swri-robotics/mapviz/wiki/mapviz.png)

Usage
-----

[View the documentation](https://swri-robotics.github.io/mapviz/) for usage information.

To acquire GPS map tiles using MapProxy, run:
```
sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

Running Mapviz in the MARINERO Project
----

To launch Mapviz manually:
```
ros2 launch mapviz mapviz.launch.py
```

However, manual launch is not required for the MARINERO project.
Mapviz is already integrated into the `marinero_navigation` package through the `localization_navigation.launch.py` file.

Additional Resources

For more details on displaying GPS maps in Mapviz (including Google Maps Satellite tiles), see: 
https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite/blob/master/README.md
