#!/usr/bin/env python3

import os
import rclpy
import math
import time
from rclpy.node import Node
import xml.etree.ElementTree as ET
import tf_transformations as tf
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
        
        self.declare_parameter("sdf_file_path", [os.path.join(package_path, "models/Marina_Zona_A/model.sdf"),
                                                os.path.join(package_path, "models/Marina_Zona_B/model.sdf"),
                                                os.path.join(package_path, "models/Marina_Zona_C/model.sdf")])

        self.declare_parameter("euler_angles", [0.0, 0.0, 3.896,
                                                0.0, 0.0, 3.8732,
                                                0.0, 0.0, 3.925
                                            ])

        self.declare_parameter("translation", [0.0, 0.0, -1.18, # 0.08,
                                                170.954, 353.30, -1.18, # 0.08, 
                                                196.9555, 650.585, -1.18, # 0.08
                                            ])

        self.labels = ["A", "B", "C"]
        self.sdf_publishers = [self.create_publisher(MarkerArray, f"/sdf_marker_{self.labels[i]}", 10) for i in range (len(self.labels))]   
        self.tf_broad = StaticTransformBroadcaster(self)

        self.sdf_file_path = list(self.get_parameter("sdf_file_path").get_parameter_value().string_array_value)
        translations_inline = self.get_parameter("translation").get_parameter_value().double_array_value
        self.translation = [translations_inline[i:i+3] for i in range(0, len(translations_inline), 3)]
        euler_inline = self.get_parameter("euler_angles").get_parameter_value().double_array_value
        euler_radians = [angle * math.pi / 180 for angle in euler_inline] 
        self.euler_angles = [euler_radians[i:i+3] for i in range(0, len(euler_radians), 3)]
        time.sleep(0.5)
        self.publish_markers()

    def publish_markers(self):
        for i in range(len(self.sdf_file_path)):
            file_path = self.sdf_file_path[i]
            frame_id = f"sdf_marker_zone_{self.labels[i]}"
            translation = self.translation[i]
            euler_angles = self.euler_angles[i]
            
            # Read SDF file
            with open(file_path, "r") as file:
                sdf_xml_string = file.read()

            # Publish SDF as Marker Array
            self.marker_array = self.create_marker_array_from_sdf(sdf_xml_string, frame_id)

            self.static_transform_publisher(frame_id, translation, euler_angles)
            self.sdf_publishers[i].publish(self.marker_array)
            self.get_logger().info(f"Publishing SDF data: {file_path}")
            time.sleep(0.5)


    def static_transform_publisher(self, frame_id, translation, euler_angles):

        rotation_angle = tf.quaternion_from_euler(euler_angles[0], euler_angles[1], euler_angles[2])

        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "map"
            t.child_frame_id = frame_id
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.x = rotation_angle[0]
            t.transform.rotation.y = rotation_angle[1]
            t.transform.rotation.z = rotation_angle[2]
            t.transform.rotation.w = rotation_angle[3]
            self.tf_broad.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Failed to publish SDF file: {e}")

    def create_marker_array_from_sdf(self, sdf_xml_string, frame_id):
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
        visual_marker.header.frame_id = frame_id
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