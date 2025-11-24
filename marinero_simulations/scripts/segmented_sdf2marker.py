#!/usr/bin/env python3

import os
import rclpy
import math
from rclpy.node import Node
import xml.etree.ElementTree as ET
import tf_transformations as tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory

package_name = "marinero_simulations"
package_path = get_package_share_directory(package_name)

class SDF2Marker(Node):

    def __init__(self):
        super().__init__("sdf_2_marker_publisher")

        self.zone_A = {
            "sdf_file_path": os.path.join(package_path, "models/Marina_Zona_A/model.sdf"),
            "euler_angles": [0.0, 0.0, 3.896],
            "translation": [0.0, 0.0, -1.18] # 0.08]
        }
        self.zone_B = {
            "sdf_file_path": os.path.join(package_path, "models/Marina_Zona_B/model.sdf"),
            "euler_angles": [0.0, 0.0, 3.8732],
            "translation": [170.954, 353.30, -1.18] # 0.08]
        }
        self.zone_C = {
            "sdf_file_path": os.path.join(package_path, "models/Marina_Zona_C/model.sdf"),
            "euler_angles": [0.0, 0.0, 3.925],
            "translation": [196.9555, 650.585, -1.18] # 0.08]
        }
        
        self.current_zone = self.zone_A
        self.sdf_published = False

        self.sdf_publisher = self.create_publisher(MarkerArray, "/sdf_markers", 10)
        self.pose_subsriber = self.create_subscription(Odometry, "/marinero/odom", self.odom_callback, 50)
        self.tf_broad = StaticTransformBroadcaster(self)

    def publish_marker(self, file_path):
            # Read SDF file
            with open(file_path, "r") as file:
                sdf_xml_string = file.read()

            # Publish SDF as Marker Array
            self.marker_array = self.create_marker_array_from_sdf(sdf_xml_string)

            self.static_transform_publisher()
            self.sdf_publisher.publish(self.marker_array)

    def odom_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y

        zone_A_limit_1, zone_A_limit_2 = 301.0, 357.45
        zone_B_limit_1, zone_B_limit_2, zone_B_limit_3 = 357.45, 661.1, 668.5
        zone_C_limit = 661.1
        zone_x_min_1, zone_x_max_1 = -23.25, -4.0
        x_pose_condition_1 = zone_x_min_1 < self.pose_x < zone_x_max_1
        zone_x_min_2, zone_x_max_2 = -45.0, -38.0
        x_pose_condition_2 = zone_x_min_2 < self.pose_x < zone_x_max_2
        
        if zone_A_limit_1 <= self.pose_y < zone_A_limit_2 and x_pose_condition_1:
            if self.current_zone != self.zone_B:
                self.switch_to_zone(self.zone_B, "Opening zone B.")
                
        elif self.pose_y < zone_A_limit_2 and self.current_zone != self.zone_A:
            self.switch_to_zone(self.zone_A, "Opening zone A.")
            
        elif zone_B_limit_1 <= self.pose_y < zone_B_limit_2 and self.current_zone != self.zone_B:
            self.switch_to_zone(self.zone_B, "Opening zone B.")
            
        elif zone_B_limit_2 <= self.pose_y < zone_B_limit_3 and x_pose_condition_2:
            if self.current_zone != self.zone_B:
                self.switch_to_zone(self.zone_B, "Opening zone B.")

        elif self.pose_y > zone_C_limit and self.current_zone != self.zone_C:
            self.switch_to_zone(self.zone_C, "Opening zone C.")

        self.declare_parameter_if_not_declared("sdf_file_path", self.current_zone["sdf_file_path"])
        self.declare_parameter_if_not_declared("euler_angles", self.current_zone["euler_angles"])
        self.declare_parameter_if_not_declared("translation", self.current_zone["translation"])

        self.sdf_file_path = self.get_parameter("sdf_file_path").get_parameter_value().string_value
        self.euler_angles = [angle * math.pi / 180 for angle in self.get_parameter("euler_angles").get_parameter_value().double_array_value]
        self.translation = self.get_parameter("translation").get_parameter_value().double_array_value
        
        if not self.sdf_published:
            self.publish_marker(self.sdf_file_path)
            self.sdf_published = True

    def switch_to_zone(self, new_zone, log_message):
        self.current_zone = new_zone
        self.sdf_published = False
        self.get_logger().info(log_message)
        
    def declare_parameter_if_not_declared(self, param_name, value):
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, value)
        else:
            self.set_parameters([Parameter(param_name, Parameter.Type.from_parameter_value(value), value)])

    def static_transform_publisher(self):
        rotation_angle = tf.quaternion_from_euler(self.euler_angles[0], self.euler_angles[1], self.euler_angles[2])
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "sdf_frame"
        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]
        t.transform.rotation.x = rotation_angle[0]
        t.transform.rotation.y = rotation_angle[1]
        t.transform.rotation.z = rotation_angle[2]
        t.transform.rotation.w = rotation_angle[3]
        self.tf_broad.sendTransform(t)

    def create_marker_array_from_sdf(self, sdf_xml_string):
        root = ET.fromstring(sdf_xml_string)

        # Create MarkerArray for visualization
        marker_array = MarkerArray()
        marker_array.markers = []
        
        # Parse visual geometry
        visual = root.find(".//visual")
        visual_pose = visual.find("pose").text.split()              # type: ignore
        visual_uri = visual.find(".//mesh/uri").text                # type: ignore
        visual_scale = visual.find(".//mesh/scale").text.split()    # type: ignore

        # Create visual marker
        visual_marker = Marker()
        visual_marker.header.frame_id = "sdf_frame"
        visual_marker.type = Marker.MESH_RESOURCE
        visual_marker.action = Marker.ADD
        visual_marker.pose.position = Point(x=0.0, 
                                            y=0.0, 
                                            z=0.0)
        visual_marker.pose.orientation.w = 1.0
        visual_marker.scale.x = float(visual_scale[0]) 
        visual_marker.scale.y = float(visual_scale[1])
        visual_marker.scale.z = float(visual_scale[2])
        visual_marker.color.r = 0.278
        visual_marker.color.g = 0.129
        visual_marker.color.b = 0.02
        visual_marker.color.a = 0.7
        visual_marker.mesh_resource = f"package:/{visual_uri}"
        visual_marker.mesh_use_embedded_materials = True
        visual_marker.id = 0
        marker_array.markers.append(visual_marker)

        return marker_array


def main(args=None):
    
    rclpy.init(args=args)
    converter = SDF2Marker()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()