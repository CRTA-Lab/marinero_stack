
import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mapviz_config = os.path.join("/home/albert/marinero_ws/src/mapviz/docs/_layouts/marinero_mapviz.mvc")

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{'config': mapviz_config}]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"local_xy_frame": "map"},
                {"local_xy_origin": "swri"},
                {"local_xy_origins": """[
                    {"name": "swri",
                        "latitude": 45.0270961,
                        "longitude": 14.625763,
                        "altitude": 1.5,
                        "heading": 0.0},
                    {"name": "back_40",
                        "latitude": 45.0270961,
                        "longitude": 14.625763,
                        "altitude": 1.5,
                        "heading": 0.0}
                ]"""},
            ],
        ),
    ])