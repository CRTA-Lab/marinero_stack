#!/usr/bin/env python3

import os
import math
import rclpy
from rclpy.node import Node
import tf_transformations as tf
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

package_name = "marinero_simulations"
package_path = get_package_share_directory(package_name)

class MarinaGazeboSpawner(Node):

    def __init__(self):
        super().__init__("marina_gazebo_spawner")
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.spawn_client.wait_for_service(timeout_sec=5.0)

        self.ZONES = {
            "A": {
                "sdf_path_1": os.path.join(package_path, "models/world_zona_A_environment/model.sdf"),
                "sdf_path_2": os.path.join(package_path, "models/world_zona_A_objects/model.sdf"),
                "euler_angles": [0.0, 0.0, 3.896], # 0.068 rad
                "translation": [96.710, 55.0214, 0.0],
                },
            "B": {
                "sdf_path_1": os.path.join(package_path, "models/world_zona_B_environment/model.sdf"),
                "sdf_path_2": os.path.join(package_path, "models/world_zona_B_objects/model.sdf"),
                "euler_angles": [0.0, 0.0, 3.8732], # 0.0676 rad
                "translation": [96.6318, 54.7578, 0.0],
                },
            "C": {
                "sdf_path_1": os.path.join(package_path, "models/world_zona_C_environment/model.sdf"),
                "sdf_path_2": os.path.join(package_path, "models/world_zona_C_objects/model.sdf"),
                "euler_angles": [0.0, 0.0, 3.925], # 0.0685 rad
                "translation": [97.16365, 54.75736, 0.0],
                }
            }
        self.spawn_zone("A")
        self.spawn_zone("B")
        self.spawn_zone("C")
        self.spawn_objects("A")
        self.spawn_objects("B") 
        self.spawn_objects("C")

    def spawn_zone(self, new_zone_label):
        new_zone = self.ZONES[new_zone_label]
        self.spawn_entity(new_zone, f"zone_{new_zone_label}")
        self.get_logger().info(f"Opened zone {new_zone_label}")

    def spawn_objects(self, new_zone_label):
        new_zone = self.ZONES[new_zone_label]
        self.spawn_entity(new_zone, f"objects_zone_{new_zone_label}", objects=True)
        self.get_logger().info(f"Opened objects in zone {new_zone_label}")

    def spawn_entity(self, zone_data, entity_name, objects=False):
        file_path = zone_data["sdf_path_2" if objects else "sdf_path_1"]
        translation = zone_data["translation"]
        euler_angles_degrees = zone_data["euler_angles"]
        euler_angles_radians = [angle * math.pi /180 for angle in euler_angles_degrees]
        rotation_angle = tf.quaternion_from_euler(euler_angles_radians[0], euler_angles_radians[1], euler_angles_radians[2])

        with open(file_path, "r") as f:
            sdf_content = f.read()

        zone_request = SpawnEntity.Request()
        zone_request.name = entity_name
        zone_request.xml = sdf_content
        zone_request.initial_pose.position.x = translation[0]
        zone_request.initial_pose.position.y = translation[1]
        zone_request.initial_pose.position.z = translation[2]
        zone_request.initial_pose.orientation.x = rotation_angle[0]
        zone_request.initial_pose.orientation.y = rotation_angle[1]
        zone_request.initial_pose.orientation.z = rotation_angle[2]
        zone_request.initial_pose.orientation.w = rotation_angle[3]

        future = self.spawn_client.call_async(zone_request)
        future.add_done_callback(self.handle_spawn_response)

    def handle_spawn_response(self, future):
        try:
            result = future.result()
            if result:
                self.get_logger().info(f"Entity spawned successfully!")
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    marina_spawner = MarinaGazeboSpawner()
    rclpy.spin(marina_spawner)
    marina_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()