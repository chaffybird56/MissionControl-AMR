#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Dict, Any


class MetricsCollector(Node):
    """
    Collects and aggregates metrics from various ROS2 topics.
    Exposes metrics via HTTP endpoint for Prometheus scraping.
    """
    
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Parameters
        self.declare_parameter('metrics_port', 8080)
        self.declare_parameter('update_rate', 1.0)
        
        # Get parameters
        self.metrics_port = self.get_parameter('metrics_port').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Metrics storage
        self.metrics = {
            'robot_pose_x': 0.0,
            'robot_pose_y': 0.0,
            'robot_pose_theta': 0.0,
            'robot_velocity_linear': 0.0,
            'robot_velocity_angular': 0.0,
            'mission_goals_completed': 0,
            'mission_total_replans': 0,
            'mission_distance_traveled': 0.0,
            'mission_success_rate': 0.0,
            'battery_level': 100.0,
            'system_uptime': 0.0,
            'active_connections': 0
        }
        
        self.metrics_lock = threading.Lock()
        self.start_time = time.time()
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.mission_metrics_sub = self.create_subscription(
            String,
            '/mission/metrics',
            self.mission_metrics_callback,
            10
        )
        
        self.mission_progress_sub = self.create_subscription(
            String,
            '/mission/progress',
            self.mission_progress_callback,
            10
        )
        
        # Timer for updating metrics
        self.metrics_timer = self.create_timer(1.0 / self.update_rate, self.update_metrics)
        
        # Start HTTP server for metrics endpoint
        self.start_metrics_server()
        
        self.get_logger().info(f'Metrics collector initialized on port {self.metrics_port}')
    
    def odom_callback(self, msg):
        """Handle odometry updates."""
        with self.metrics_lock:
            self.metrics['robot_pose_x'] = msg.pose.pose.position.x
            self.metrics['robot_pose_y'] = msg.pose.pose.position.y
            self.metrics['robot_pose_theta'] = self.quaternion_to_yaw(msg.pose.pose.orientation)
            self.metrics['robot_velocity_linear'] = msg.twist.twist.linear.x
            self.metrics['robot_velocity_angular'] = msg.twist.twist.angular.z
    
    def mission_metrics_callback(self, msg):
        """Handle mission metrics updates."""
        try:
            metrics_data = json.loads(msg.data)
            with self.metrics_lock:
                self.metrics['mission_goals_completed'] = metrics_data.get('goals_completed', 0)
                self.metrics['mission_total_replans'] = metrics_data.get('total_replans', 0)
                self.metrics['mission_distance_traveled'] = metrics_data.get('distance_traveled', 0.0)
                self.metrics['mission_success_rate'] = metrics_data.get('success_rate', 0.0)
        except json.JSONDecodeError:
            pass
    
    def mission_progress_callback(self, msg):
        """Handle mission progress updates."""
        try:
            progress_data = json.loads(msg.data)
            # Extract additional metrics from progress data if needed
        except json.JSONDecodeError:
            pass
    
    def update_metrics(self):
        """Update system metrics."""
        with self.metrics_lock:
            self.metrics['system_uptime'] = time.time() - self.start_time
            # Mock battery level (would be from actual sensor in real implementation)
            self.metrics['battery_level'] = max(0, 100 - (time.time() - self.start_time) * 0.1)
    
    def start_metrics_server(self):
        """Start HTTP server for metrics endpoint."""
        class MetricsHandler(BaseHTTPRequestHandler):
            def __init__(self, metrics_collector, *args, **kwargs):
                self.metrics_collector = metrics_collector
                super().__init__(*args, **kwargs)
            
            def do_GET(self):
                if self.path == '/metrics':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/plain')
                    self.end_headers()
                    
                    metrics_text = self.metrics_collector.get_prometheus_metrics()
                    self.wfile.write(metrics_text.encode())
                else:
                    self.send_response(404)
                    self.end_headers()
            
            def log_message(self, format, *args):
                # Suppress default logging
                pass
        
        def handler_factory(*args, **kwargs):
            return MetricsHandler(self, *args, **kwargs)
        
        def run_server():
            server = HTTPServer(('0.0.0.0', self.metrics_port), handler_factory)
            server.serve_forever()
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
    
    def get_prometheus_metrics(self) -> str:
        """Generate Prometheus-formatted metrics."""
        with self.metrics_lock:
            metrics_lines = []
            
            # Robot position metrics
            metrics_lines.append(f'# HELP robot_pose_x Robot X position in meters')
            metrics_lines.append(f'# TYPE robot_pose_x gauge')
            metrics_lines.append(f'robot_pose_x {self.metrics["robot_pose_x"]}')
            
            metrics_lines.append(f'# HELP robot_pose_y Robot Y position in meters')
            metrics_lines.append(f'# TYPE robot_pose_y gauge')
            metrics_lines.append(f'robot_pose_y {self.metrics["robot_pose_y"]}')
            
            metrics_lines.append(f'# HELP robot_pose_theta Robot orientation in radians')
            metrics_lines.append(f'# TYPE robot_pose_theta gauge')
            metrics_lines.append(f'robot_pose_theta {self.metrics["robot_pose_theta"]}')
            
            # Velocity metrics
            metrics_lines.append(f'# HELP robot_velocity_linear Robot linear velocity in m/s')
            metrics_lines.append(f'# TYPE robot_velocity_linear gauge')
            metrics_lines.append(f'robot_velocity_linear {self.metrics["robot_velocity_linear"]}')
            
            metrics_lines.append(f'# HELP robot_velocity_angular Robot angular velocity in rad/s')
            metrics_lines.append(f'# TYPE robot_velocity_angular gauge')
            metrics_lines.append(f'robot_velocity_angular {self.metrics["robot_velocity_angular"]}')
            
            # Mission metrics
            metrics_lines.append(f'# HELP mission_goals_completed Total number of goals completed')
            metrics_lines.append(f'# TYPE mission_goals_completed counter')
            metrics_lines.append(f'mission_goals_completed {self.metrics["mission_goals_completed"]}')
            
            metrics_lines.append(f'# HELP mission_total_replans Total number of path replans')
            metrics_lines.append(f'# TYPE mission_total_replans counter')
            metrics_lines.append(f'mission_total_replans {self.metrics["mission_total_replans"]}')
            
            metrics_lines.append(f'# HELP mission_distance_traveled Total distance traveled in meters')
            metrics_lines.append(f'# TYPE mission_distance_traveled counter')
            metrics_lines.append(f'mission_distance_traveled {self.metrics["mission_distance_traveled"]}')
            
            metrics_lines.append(f'# HELP mission_success_rate Mission success rate (0.0 to 1.0)')
            metrics_lines.append(f'# TYPE mission_success_rate gauge')
            metrics_lines.append(f'mission_success_rate {self.metrics["mission_success_rate"]}')
            
            # System metrics
            metrics_lines.append(f'# HELP battery_level Robot battery level percentage')
            metrics_lines.append(f'# TYPE battery_level gauge')
            metrics_lines.append(f'battery_level {self.metrics["battery_level"]}')
            
            metrics_lines.append(f'# HELP system_uptime System uptime in seconds')
            metrics_lines.append(f'# TYPE system_uptime counter')
            metrics_lines.append(f'system_uptime {self.metrics["system_uptime"]}')
            
            metrics_lines.append(f'# HELP active_connections Number of active WebSocket connections')
            metrics_lines.append(f'# TYPE active_connections gauge')
            metrics_lines.append(f'active_connections {self.metrics["active_connections"]}')
            
            return '\n'.join(metrics_lines) + '\n'
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        import math
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw


def main(args=None):
    rclpy.init(args=args)
    
    metrics_collector = MetricsCollector()
    
    try:
        rclpy.spin(metrics_collector)
    except KeyboardInterrupt:
        pass
    finally:
        metrics_collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

