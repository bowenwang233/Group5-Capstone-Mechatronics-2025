#!/usr/bin/env python3
"""
Control Subscriber for Physical Twin
=====================================
ROS2 node that receives velocity commands via ZeroMQ and publishes to /cmd_vel.

This node:
- Binds a ZeroMQ SUB socket on tcp://*:5558
- Receives JSON velocity commands {linear, angular, timestamp}
- Publishes Twist messages to /cmd_vel topic
- Implements safety timeout (stops if no command received for 1 second)

Usage:
    ros2 run <package> ControlSubscriber.py
    or
    python3 ControlSubscriber.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import zmq
import json
import time
import threading


class ControlSubscriber(Node):
    def __init__(self):
        super().__init__('control_subscriber')
        
        # Declare parameters
        self.declare_parameter('bind_port', 5558)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('safety_timeout', 1.0)  # seconds
        
        # Get parameters
        bind_port = self.get_parameter('bind_port').get_parameter_value().integer_value
        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        pub_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.safety_timeout = self.get_parameter('safety_timeout').get_parameter_value().double_value
        
        # Initialize ZeroMQ
        self.zmq_ctx = zmq.Context()
        self.zmq_sock = self.zmq_ctx.socket(zmq.SUB)
        self.zmq_sock.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.zmq_sock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        bind_addr = f"tcp://*:{bind_port}"
        self.zmq_sock.bind(bind_addr)
        
        # ROS2 publisher
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        
        # State
        self.current_cmd = Twist()
        self.last_cmd_time = time.time()
        self.cmd_lock = threading.Lock()
        
        # Start ZMQ receiver thread
        self.running = True
        self.zmq_thread = threading.Thread(target=self._zmq_loop, daemon=True)
        self.zmq_thread.start()
        
        # ROS2 timer for publishing
        timer_period = 1.0 / pub_rate
        self.timer = self.create_timer(timer_period, self._publish_callback)
        
        self.get_logger().info(
            f"[ControlSubscriber] Started\n"
            f"  ZMQ Bind: {bind_addr}\n"
            f"  ROS2 Topic: {cmd_topic}\n"
            f"  Publish Rate: {pub_rate} Hz\n"
            f"  Safety Timeout: {self.safety_timeout} s"
        )
    
    def _zmq_loop(self):
        """Background thread to receive ZMQ commands."""
        while self.running:
            try:
                msg_json = self.zmq_sock.recv_json()
                
                # Parse command
                linear = float(msg_json.get('linear', 0.0))
                angular = float(msg_json.get('angular', 0.0))
                
                # Update current command
                with self.cmd_lock:
                    self.current_cmd.linear.x = linear
                    self.current_cmd.angular.z = angular
                    self.last_cmd_time = time.time()
                
                self.get_logger().debug(
                    f"Received: linear={linear:.2f}, angular={angular:.2f}"
                )
                
            except zmq.Again:
                # Timeout, no message received - this is normal
                continue
            except Exception as e:
                self.get_logger().error(f"ZMQ receive error: {e}")
                time.sleep(0.1)
    
    def _publish_callback(self):
        """ROS2 timer callback - publishes current command or stop if timed out."""
        with self.cmd_lock:
            time_since_cmd = time.time() - self.last_cmd_time
            
            # Safety check: if no command for safety_timeout, send stop
            if time_since_cmd > self.safety_timeout:
                if self.current_cmd.linear.x != 0.0 or self.current_cmd.angular.z != 0.0:
                    self.get_logger().warn(
                        f"No command for {time_since_cmd:.1f}s - Sending STOP"
                    )
                    self.current_cmd.linear.x = 0.0
                    self.current_cmd.angular.z = 0.0
            
            # Publish current command
            self.cmd_pub.publish(self.current_cmd)
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info("[ControlSubscriber] Shutting down...")
        
        # Stop ZMQ thread
        self.running = False
        if self.zmq_thread.is_alive():
            self.zmq_thread.join(timeout=2.0)
        
        # Send final stop command
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        
        # Close ZMQ
        try:
            self.zmq_sock.close(linger=0)
            self.zmq_ctx.term()
        except Exception as e:
            self.get_logger().error(f"Error closing ZMQ: {e}")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

