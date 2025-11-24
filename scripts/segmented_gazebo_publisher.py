#!/usr/bin/env python3

import os
import sys
import math
import rclpy
from rclpy.node import Node
import tf_transformations as tf
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from ament_index_python.packages import get_package_share_directory

package_name = "marinero_simulations"
package_path = get_package_share_directory(package_name)

class GazeboSpawner(Node):

    def __init__(self, x_pose, y_pose):
        super().__init__("gazebo_spawner")

        # self.pose_subscriber = self.create_subscription(PoseStamped, "/robot_pose", self.pose_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/marinero/odom", self.odom_callback, 10)
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")

        self.spawn_client.wait_for_service(timeout_sec=30.0)
        self.delete_client.wait_for_service(timeout_sec=30.0)

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

        self.current_zone = None
        self.previous_zone = None

        self.initialize_zones(y_pose)

    def initialize_zones(self, y_pose):
        if y_pose < 301.0:
            new_zone_label = "A"
        elif 301.0 <= y_pose < 668.5:
            new_zone_label = "B"
        else:
            new_zone_label = "C"

        self.spawn_entity(self.ZONES[new_zone_label], f"zone_{new_zone_label}")
        self.spawn_entity(self.ZONES[new_zone_label], f"objects_zone_{new_zone_label}", objects=True)
        self.get_logger().info(f"Inital zone {new_zone_label} generated.")
        self.current_zone = new_zone_label

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        zone_A_limit_1, zone_A_limit_2 = 301.0, 357.45
        zone_B_limit_1, zone_B_limit_2, zone_B_limit_3 = 357.45, 661.1, 668.5
        zone_C_limit = 661.1
        zone_x_min_1, zone_x_max_1 = -23.25, -4.0
        x_pose_condition_1 = zone_x_min_1 < self.pose_x < zone_x_max_1
        zone_x_min_2, zone_x_max_2 = -46.0, -38.0
        x_pose_condition_2 = zone_x_min_2 < self.pose_x < zone_x_max_2
        
        if zone_A_limit_1 <= self.pose_y < zone_A_limit_2 and x_pose_condition_1:
            if self.current_zone != "B":
                self.switch_zone("B")
        elif self.pose_y < zone_A_limit_2 and self.current_zone != "A":
            self.switch_zone("A")
        elif zone_B_limit_1 <= self.pose_y < zone_B_limit_2 and self.current_zone != "B":
            self.switch_zone("B")
        elif zone_B_limit_2 <= self.pose_y < zone_B_limit_3 and x_pose_condition_2:
            if self.current_zone != "B":
                self.switch_zone("B")
        elif self.pose_y > zone_C_limit and self.current_zone != "C":
            self.switch_zone("C")
        self.previous_zone = self.current_zone


    def switch_zone(self, new_zone_label):
        
        if self.current_zone == new_zone_label:
            return
        
        new_zone = self.ZONES[new_zone_label]
        self.spawn_entity(new_zone, f"zone_{new_zone_label}")
        self.spawn_entity(new_zone, f"objects_zone_{new_zone_label}", objects=True)
        
        if self.previous_zone:
            self.delete_entity(f"objects_zone_{self.previous_zone}")
            self.delete_entity(f"zone_{self.previous_zone}")
            
        self.current_zone = new_zone_label


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
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {str(e)}")


    def delete_entity(self, entity_name):
        delete_request = DeleteEntity.Request()
        delete_request.name = entity_name
        
        future = self.delete_client.call_async(delete_request)
        future.add_done_callback(self.handle_delete_response)


    def handle_delete_response(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Delete failed: {str(e)}")

def main(args=None):

    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run marinero_simulations segmented_gazebo_publisher.py <x_pose> <y_pose>")
        sys.exit(1)
        
    x_pose = float(sys.argv[1])
    y_pose = float(sys.argv[2])
    
    gazebo_spawner = GazeboSpawner(x_pose, y_pose)
    
    rclpy.spin(gazebo_spawner)
    gazebo_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()