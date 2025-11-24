#!/usr/bin/env python3

import os
import yaml
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# from location_interfaces.srv import ExecuteMission
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32, Bool
from nav2_msgs.action import NavigateToPose, FollowWaypoints

class GoalExecutor(Node):
    def __init__(self):
        super().__init__('goal_executor')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.current_goal_handle = None
        self.current_file = None
        self.coordinates = []
        self.current_index = 0
        self.mission_active = False
        self.proceed_with_goals = False

        self.goal_location_publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.process_status_publisher = self.create_publisher(String, "/process_status", 10)
        self.mission_status_publisher = self.create_publisher(Bool, "/mission_status", 10)
        self.success_status_publisher = self.create_publisher(Int32, "/success_status", 10)

        self.abort_subscription = self.create_subscription(String, "/abort_mission", self.abort_mission_callback, 10)
        self.proceed_with_goals_subscription = self.create_subscription (Bool, "/proceed_with_goals", self.proceed_with_goals_callback, 10)

        self.create_timer(1.0, self.check_for_mission)
        self.get_logger().info("Goal Executor Node started - waiting for missions")


    def proceed_with_goals_callback(self, msg):
        self.proceed_with_goals = msg.data
        if self.proceed_with_goals and self.current_index != 0:
            self.execute_next_goal()


    def abort_mission_callback(self, msg):
        if msg.data:
            self.process_status_publisher.publish(String(data="Mission aborted by user"))
            if self.current_goal_handle is not None:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda future: self.get_logger().info("Current goal canceled"))
            self.cleanup_mission()


    def check_for_mission(self):
        if self.mission_active:
            return

        for file in os.listdir('.'):
            if file.endswith('.yaml') and file.startswith('coordinates'):
                self.start_mission(file)
                return


    def start_mission(self, filename):
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                if not data or 'coordinates' not in data:
                    self.get_logger().error("Invalid YAML format")
                    return

                self.coordinates = data['coordinates']
                self.current_index = 0
                self.current_file = filename
                self.mission_active = True
                self.mission_status_publisher.publish(Bool(data=True))
                # self.get_logger().info(f"Starting mission with {len(self.coordinates)} goals from {filename}")
                self.process_status_publisher.publish(String(data=f"Starting mission with {len(self.coordinates)} goals"))
                self.execute_next_goal()

        except Exception as e:
            self.get_logger().error(f"Error loading mission file: {str(e)}")


    def execute_next_goal(self):
        if self.current_index >= len(self.coordinates):
            self.get_logger().info("Mission complete!")
            self.cleanup_mission()
            return

        coord = self.coordinates[self.current_index]
        self.current_index += 1
        self.send_goal(coord, publish=True, index=self.current_index)


    def retry_goal(self):
        if self.current_index == 0 or self.current_index > len(self.coordinates):
            return

        coord = self.coordinates[self.current_index - 1]
        self.send_goal(coord, publish=True, index=self.current_index)


    def send_goal(self, coord, publish = True, index = None):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coord['x']
        goal_msg.pose.pose.position.y = coord['y']
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz = 0.0, 0.0 , coord['yaw']
        qw = math.sqrt(max(0.0, 1.0 - qx**2 - qy**2 - qz**2))

        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        if publish:
            self.goal_location_publisher.publish(goal_msg.pose)
            if index:
                self.process_status_publisher.publish(String(data=f"Sending goal {index} at ({coord['x']}, {coord['y']})"))

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.process_status_publisher.publish(String(data=f"Goal {self.current_index}. of {len(self.coordinates)} rejected"))
            self.execute_next_goal()
            return

        self.current_goal_handle = goal_handle
        self.process_status_publisher.publish(String(data=f"Goal {self.current_index}. of {len(self.coordinates)} accepted, waiting for result..."))
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)


    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.process_status_publisher.publish(String(data=f"Robot has reached {self.current_index}. of {len(self.coordinates)} goals!\n"
                                                            f"Waiting for boat photograph..."))
        elif status == 5:
            self.process_status_publisher.publish(String(data=f"Mission canceled!"))
        else:
            self.process_status_publisher.publish(String(data=f"Failed to reach {self.current_index}. of {len(self.coordinates)} goals, with status {status}!\n"
                                                            f"Retrying..."))
            self.retry_goal()
            
        self.success_status_publisher.publish(Int32(data=status))


    def cleanup_mission(self):
        try:
            if self.current_file is not None:
                os.remove(self.current_file)
                self.get_logger().info(f"Removed mission file: {self.current_file}")
        except Exception as e:
            self.get_logger().error(f"Error removing mission file: {str(e)}")

        self.current_file = None
        self.coordinates = []
        self.current_index = 0
        self.mission_active = False
        self.mission_status_publisher.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    executor = GoalExecutor()
    multi_exec = MultiThreadedExecutor()
    multi_exec.add_node(executor)
    
    try:
        multi_exec.spin()
    except KeyboardInterrupt:
        executor.get_logger().info("Shutting down")
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
