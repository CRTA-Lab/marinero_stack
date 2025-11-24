#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class Pose_Z_Offset(Node):
    def __init__(self):
        super().__init__("pose_z_offset")
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.estimate_pose_subsriber = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.estimate_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.marina_dock_height = 1.26 # m
        self.goal_published_flag = False
        self.estimate_pose_published_flag = False
        self.previous_goal_pose = None
        self.previous_estimated_pose = None


    def goal_callback(self, msg):

        initial_goal_pose = msg
        if initial_goal_pose != self.previous_goal_pose:
            self.goal_published_flag = False

        if not self.goal_published_flag:
            goal_pose = PoseStamped()
            goal_pose.header = Header()
            goal_pose.header.frame_id = "world"
            goal_pose.pose.position.x = initial_goal_pose.pose.position.x
            goal_pose.pose.position.y = initial_goal_pose.pose.position.y
            goal_pose.pose.position.z = self.marina_dock_height
            goal_pose.pose.orientation = initial_goal_pose.pose.orientation
            self.goal_pose_publisher.publish(goal_pose)
            self.goal_published_flag = True
            self.previous_goal_pose = goal_pose

    def pose_callback(self, msg):
        
        initial_estimate_pose = msg
        if initial_estimate_pose != self.previous_estimated_pose:
            self.estimate_pose_published_flag = False
        
        if not self.estimate_pose_published_flag:
            estimate_pose = PoseWithCovarianceStamped()
            estimate_pose.header = Header()
            estimate_pose.header.frame_id = "world"
            estimate_pose.pose.pose.position.x = initial_estimate_pose.pose.pose.position.x
            estimate_pose.pose.pose.position.y = initial_estimate_pose.pose.pose.position.y
            estimate_pose.pose.pose.position.z = self.marina_dock_height
            estimate_pose.pose.pose.orientation = initial_estimate_pose.pose.pose.orientation
            self.estimate_pose_publisher.publish(estimate_pose)
            self.estimate_pose_published_flag = True
            self.previous_estimated_pose = estimate_pose


def main(args=None):
    rclpy.init(args=args)
    node = Pose_Z_Offset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()