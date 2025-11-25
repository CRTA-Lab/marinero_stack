#!/usr/bin/env python3

import os
import time
import socket
import threading
from PIL import Image
from pathlib import Path
from http.server import SimpleHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool


photo_path = Path("/home/albert/marinero_ws/waypoint_images") # Hardcoded to my PC workstation --> modify to your needs
port = 7050


class TCPBridgeNode(Node):
    def __init__(self):
        super().__init__('tcp_bridge_node')

        self.declare_parameter('listen_host', '192.168.42.134')
        self.declare_parameter('listen_port', 7020)
        self.declare_parameter('send_host', '10.182.42.133')
        self.declare_parameter('send_port', 7020)
        self.declare_parameter('confirmation_message', 'Message received!')
        self.declare_parameter('receive_timeout', 1.0)
        self.declare_parameter('send_timeout', 1.0)
        self.declare_parameter('topic_name', 'coordinates_topic')

        self.listen_host = self.get_parameter('listen_host').value
        self.listen_port = self.get_parameter('listen_port').value
        self.send_host = self.get_parameter('send_host').value
        self.send_port = self.get_parameter('send_port').value
        self.receive_timeout = self.get_parameter('receive_timeout').value
        self.send_timeout = self.get_parameter('send_timeout').value

        self.photo_saved = 0        # 0 = travelling, 1 = photo saved, 2 = photo not sent, 3 = home
        self.success_status = 0
        self.listener_socket = None
        self.active_connection = None
        self.shutdown_requested = False
        self.mission_active = False

        confirmation_message_value = self.get_parameter('confirmation_message').value
        if confirmation_message_value is None:
            confirmation_message_value = 'Message not received!'
        self.confirmation_msg = confirmation_message_value

        topic_name = self.get_parameter('topic_name').value
        if not isinstance(topic_name, str) or topic_name is None:
            topic_name = 'coordinates_topic'
        self.publisher_ = self.create_publisher(String, topic_name, 10)

        self.mission_status_subscription = self.create_subscription(Bool, "/mission_status", self.mission_status_callback, 10)
        self.process_status_subscription = self.create_subscription(String, "/process_status", self.process_status_callback, 10)
        self.photo_saved_subscription = self.create_subscription(Int32, "/photo_saved", self.photo_saved_callback, 10)
        self.success_status_subscription = self.create_subscription(Int32, "/success_status", self.success_status_callback, 10)
        self.proceed_with_goals_publisher = self.create_publisher(Bool, "/proceed_with_goals", 10)

        self.timer = self.create_timer(0.1, self.main_loop)

        # ---- Start HTTP server in background thread ----
        threading.Thread(target=self.start_http_server, daemon=True).start()
        self.setup_listener()

        self.get_logger().info(f'Node started, listening on {self.listen_host}:{self.listen_port}')
        self.get_logger().info(f'Publishing messages to topic: {self.get_parameter("topic_name").value}')


    def success_status_callback(self, msg):
        self.success_status = msg.data

    def mission_status_callback(self, msg):
        self.mission_active = msg.data

    def process_status_callback(self, msg):
        self.send_message(msg.data)

    def photo_saved_callback(self, msg):
        self.photo_saved = msg.data
        self.prepare_photo()

    def start_http_server(self):
        os.makedirs(photo_path, exist_ok=True)
        os.chdir(photo_path)
        server = HTTPServer(("0.0.0.0", port), SimpleHTTPRequestHandler)
        print(f"Serving photos at: http://{self.listen_host}:{port}/latest.png")
        server.serve_forever()

    def setup_listener(self):
        try:
            self.set_socket()
        except Exception as e:
            self.get_logger().error(f'Error setting up listener: {str(e)}')
            time.sleep(1)
            if not self.shutdown_requested:
                self.setup_listener()


    def set_socket(self):
        if self.listener_socket:
            self.listener_socket.close()

        self.listener_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener_socket.bind((self.listen_host, self.listen_port))
        self.listener_socket.listen()
        self.listener_socket.settimeout(self.receive_timeout)


    def send_message(self, message: str):
        try:
            if self.active_connection:
                self.active_connection.sendall((message + "\n").encode("utf-8"))
                # self.get_logger().info(f'Successfully sent message to client', throttle_duration_sec=1)
            else:
                self.get_logger().warn("No active connection to send message")
        except Exception as e:
            self.get_logger().error(f'Error sending message: {str(e)}')


    def prepare_photo(self):
        try:
            send_message = True
            if not self.active_connection and send_message:
                self.get_logger().warn("No active connection to send message")
                send_message = False
                return
            if not self.mission_active and send_message:
                self.get_logger().info("Mission inactive — not sending photo message")
                send_message = False
                return
            if self.success_status == 4:
                if self.photo_saved == 1:
                    if not photo_path.exists():
                        self.get_logger().error(f"Photo path does not exist: {photo_path}")
                        msg = f"Photo path does not exist: {photo_path}"
                    else:
                        png_files = list(photo_path.glob("*.png"))
                        if not png_files:
                            msg = "No PNG photo files found in directory"
                        else:
                            latest_file = max(png_files, key=lambda f: f.stat().st_mtime)
                            img = Image.open(latest_file)
                            img.thumbnail((640, 480))
                            img.save(photo_path / "latest.png", format="PNG")
                            msg = f"Photo ready at: http://{self.listen_host}:{port}/latest.png"
                elif self.photo_saved == 2:
                    msg = "Photo not saved, boat not detected!"
                elif self.photo_saved == 3:
                    msg = "Robot at HOME, photo not required"
                else:
                    return
                self.send_message(msg)
                self.proceed_with_goals_publisher.publish(Bool(data=True))

        except Exception as e:
            self.get_logger().error(f'Error preparing photo: {str(e)}')


    def publish_message(self, data):
        try:
            msg = String()
            msg.data = data.decode() if isinstance(data, bytes) else str(data)
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published to topic: {msg.data}', throttle_duration_sec=1)
        except Exception as e:
            self.get_logger().error(f'Error publishing message: {str(e)}')


    def main_loop(self):
        if self.shutdown_requested:
            return

        try:
            if self.listener_socket is None:
                return
            if self.active_connection is None:
                # Wait for new client
                conn, addr = self.listener_socket.accept()
                self.get_logger().info(f"New connection from {addr[0]}:{addr[1]}")
                conn.settimeout(self.receive_timeout)
                self.active_connection = conn

            # Try receiving from active connection
            try:
                data = self.active_connection.recv(4096)
                if not data:
                    self.get_logger().warn("Client disconnected")
                    self.active_connection.close()
                    self.active_connection = None
                    return

                # self.get_logger().info(f"Received: {data.decode(errors='ignore')}")
                # Send confirmation back
                self.send_message(self.confirmation_msg)
                # Publish to ROS
                self.publish_message(data)

            except socket.timeout:
                pass
            except Exception as e:
                self.get_logger().error(f'Connection error: {str(e)}')
                if self.active_connection:
                    self.active_connection.close()
                    self.active_connection = None

        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f'Accept error: {str(e)}')
            if not self.shutdown_requested:
                time.sleep(1.0)
                self.setup_listener()


    def shutdown(self):
        self.shutdown_requested = True
        self.get_logger().info("Shutting down node...")
        
        if self.active_connection:
            try:
                self.active_connection.close()
            except:
                pass
                
        if self.listener_socket:
            try:
                self.listener_socket.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = TCPBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        node.get_logger().info("Node shutdown complete")

if __name__ == '__main__':
    main()
