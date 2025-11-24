#!/usr/bin/env python3

import os
import xml.etree.ElementTree as ET
import rclpy
import math
import tf_transformations as tf
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SdfToUrdfConverter(Node):

    def __init__(self, file_path, output_urdf_file_path):
        super().__init__("sdf_to_urdf_converter")
        self.output_urdf_file_path = output_urdf_file_path
        self.declare_parameter("euler_angles", [0.0, 0.0, 0.0])         # in degrees
        # self.declare_parameter("translation", [-100.0, -48.0, 0.08])  # for the other sdf model written in line 180
        self.declare_parameter("translation", [-100.0, -48.0, 0.08])
        self.euler_angles = [angle * math.pi / 180 for angle in self.get_parameter("euler_angles").get_parameter_value().double_array_value]
        self.translation = self.get_parameter("translation").get_parameter_value().double_array_value

        self.sdf_publisher = self.create_publisher(MarkerArray, "/sdf_markers", 10) 
        self.counter = 0       
        self.br = TransformBroadcaster(self)
        self.tf_broad = StaticTransformBroadcaster(self)
        
        
        # Read SDF file
        with open(file_path, "r") as file:
            sdf_xml_string = file.read()

        # Convert SDF to URDF and publish Marker Array
        urdf_string = self.convert_sdf_to_urdf(sdf_xml_string)
        self.marker_array = self.create_marker_array_from_sdf(sdf_xml_string)

        # # Print URDF string
        # self.get_logger().info("Generated URDF.")
        
        # # Write URDF to file
        # with open(self.output_urdf_file_path, "w") as urdf_file:
        #     urdf_file.write(urdf_string)


        # Publish Marker Array
        self.marina_timer = self.create_timer(5.0, self.publish_marker_array)

    def convert_sdf_to_urdf(self, sdf_xml_string):
        # sourcery skip: use-fstring-for-concatenation
        root = ET.fromstring(sdf_xml_string)

        # Extract model name
        model_name = root.find('model').get('name') # type: ignore

        urdf_string = '<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="' + model_name + '">\n\n'    # type: ignore
        
        # Add visual geometry
        visual = root.find('.//visual')
        visual_pose = visual.find('pose').text.split()              # type: ignore
        visual_uri = visual.find('.//mesh/uri').text                # type: ignore
        visual_scale = visual.find('.//mesh/scale').text.split()    # type: ignore
        
        # Add collision geometry
        collision = root.find('.//collision')
        collision_pose = collision.find('pose').text.split()            # type: ignore
        collision_uri = collision.find('.//mesh/uri').text              # type: ignore
        collision_scale = collision.find('.//mesh/scale').text.split()  # type: ignore

        # Add inertial properties
        inertial = root.find('.//inertial')
        mass = float(inertial.find('mass').text)                    # type: ignore
        ix = float(inertial.find('.//inertia/ixx').text)            # type: ignore
        iy = float(inertial.find('.//inertia/iyy').text)            # type: ignore
        iz = float(inertial.find('.//inertia/izz').text)            # type: ignore
        
        urdf_string += '  <link name="odom"></link>\n\n'
        
        urdf_string += '  <joint name="' + model_name + '_joint" type="fixed">\n'       # type: ignore
        urdf_string += '    <origin xyz="' + str(self.translation[0]) + ' ' + str(self.translation[1]) + ' ' + str(self.translation[2]) + '" rpy="0.0 0.0 0.0" />\n'
        urdf_string += '    <parent link="odom" />\n'
        urdf_string += '    <child link="' + model_name + '_link' + '" />\n'    # type: ignore
        urdf_string += '  </joint>\n\n'
        
        urdf_string += '  <link name="' + model_name + '">\n'                   # type: ignore
        urdf_string += '    <visual name="visual">\n'
        urdf_string += '      <geometry>\n'
        urdf_string += '        <mesh filename="' + os.path.abspath(visual_uri) + '"/>\n'           # type: ignore
        urdf_string += '      </geometry>\n'
        urdf_string += '      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>\n'
        urdf_string += '      <material name="Gazebo/Brown">\n'
        urdf_string += '        <color rgba="0.278 0.129 0.02 1"/>\n'
        urdf_string += '      </material>\n'
        urdf_string += '    </visual>\n\n'

        urdf_string += '    <collision name="collision">\n'
        urdf_string += '      <geometry>\n'
        urdf_string += '        <mesh filename="' + os.path.abspath(collision_uri) + '"/>\n'      # type: ignore  
        urdf_string += '      </geometry>\n'
        urdf_string += '      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>\n'
        urdf_string += '    </collision>\n\n'
        
        urdf_string += '    <inertial>\n'
        urdf_string += '      <mass value="' + str(mass) + '"/>\n'
        urdf_string += '      <inertia ixx="' + str(ix) + '" ixy="0" ixz="0" iyy="' + str(iy) + '" iyz="0" izz="' + str(iz) + '"/>\n'
        urdf_string += '    </inertial>\n\n'

        urdf_string += '  </link>\n'
        
        urdf_string += '</robot>\n'
        
        return urdf_string

    def static_transform_publisher(self):
        rotation_angle = tf.quaternion_from_euler(self.euler_angles[0], self.euler_angles[1], self.euler_angles[2])
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'sdf_frame'
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

        # Parse visual geometry
        visual = root.find('.//visual')
        visual_pose = visual.find('pose').text.split()              # type: ignore
        visual_uri = visual.find('.//mesh/uri').text                # type: ignore
        visual_scale = visual.find('.//mesh/scale').text.split()    # type: ignore

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
        visual_marker.color.a = 0.8
        visual_marker.mesh_resource = "file://" + os.path.abspath(visual_uri)       # type: ignore
        visual_marker.mesh_use_embedded_materials = True
        visual_marker.id = 0
        marker_array.markers.append(visual_marker)      # type: ignore

        return marker_array
    
    def publish_marker_array(self):
        if self.counter < 10:
            self.static_transform_publisher()
            self.sdf_publisher.publish(self.marker_array)
            self.counter += 1

def main(args=None):
    
    rclpy.init(args=args)

    # Replace with the path to your SDF file
    # sdf_file_path = "/home/albert/marinero_ws/src/marinero_simulations/models/Marina_dokovi/model.sdf"
    sdf_file_path = "/home/albert/marinero_ws/src/marinero_simulations/models/Marina_Zona_A/model.sdf"
    output_urdf_file_path = "/home/albert/marinero_ws/src/marinero_simulations/robot_description/marina.urdf.xacro"

    converter = SdfToUrdfConverter(sdf_file_path, output_urdf_file_path)
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()