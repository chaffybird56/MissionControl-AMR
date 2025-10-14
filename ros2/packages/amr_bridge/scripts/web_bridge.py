#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
import json
import asyncio
import websockets
import threading
import time
from typing import Set


class WebBridge(Node):
    """
    WebSocket bridge for real-time communication between ROS2 and web dashboard.
    Handles telemetry streaming and command reception.
    """
    
    def __init__(self):
        super().__init__('web_bridge')
        
        # Parameters
        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('max_connections', 10)
        self.declare_parameter('telemetry_rate', 10.0)
        
        # Get parameters
        self.websocket_port = self.get_parameter('websocket_port').value
        self.max_connections = self.get_parameter('max_connections').value
        self.telemetry_rate = self.get_parameter('telemetry_rate').value
        
        # WebSocket connections
        self.connections: Set[websockets.WebSocketServerProtocol] = set()
        
        # Robot state
        self.current_pose = None
        self.current_velocity = None
        self.battery_level = 100.0  # Mock battery
        self.mission_state = "idle"
        self.goal_state = "none"
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.mission_state_sub = self.create_subscription(
            String,
            '/mission/state',
            self.mission_state_callback,
            10
        )
        
        self.mission_progress_sub = self.create_subscription(
            String,
            '/mission/progress',
            self.mission_progress_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(String, '/emergency_stop', 10)
        
        # Service clients
        self.start_mission_client = self.create_client(Empty, '/mission/start')
        self.stop_mission_client = self.create_client(Empty, '/mission/stop')
        self.pause_mission_client = self.create_client(Empty, '/mission/pause')
        
        # Timer for telemetry
        self.telemetry_timer = self.create_timer(1.0 / self.telemetry_rate, self.broadcast_telemetry)
        
        # Start WebSocket server in a separate thread
        self.start_websocket_server()
        
        self.get_logger().info(f'Web bridge initialized on port {self.websocket_port}')
    
    def start_websocket_server(self):
        """Start the WebSocket server in a separate thread."""
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            start_server = websockets.serve(
                self.handle_connection,
                "0.0.0.0",
                self.websocket_port,
                max_size=1024*1024,  # 1MB max message size
                ping_interval=20,
                ping_timeout=10
            )
            
            loop.run_until_complete(start_server)
            loop.run_forever()
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
    
    async def handle_connection(self, websocket, path):
        """Handle new WebSocket connections."""
        if len(self.connections) >= self.max_connections:
            await websocket.close(code=1013, reason="Server overloaded")
            return
        
        self.connections.add(websocket)
        self.get_logger().info(f'New WebSocket connection: {websocket.remote_address}')
        
        try:
            # Send initial state
            await self.send_telemetry_to_client(websocket)
            
            # Handle messages from client
            async for message in websocket:
                await self.handle_client_message(websocket, message)
                
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connections.discard(websocket)
            self.get_logger().info(f'WebSocket connection closed: {websocket.remote_address}')
    
    async def handle_client_message(self, websocket, message):
        """Handle incoming messages from WebSocket clients."""
        try:
            data = json.loads(message)
            command = data.get('command')
            
            if command == 'start_mission':
                await self.handle_start_mission()
            elif command == 'stop_mission':
                await self.handle_stop_mission()
            elif command == 'pause_mission':
                await self.handle_pause_mission()
            elif command == 'emergency_stop':
                await self.handle_emergency_stop()
            elif command == 'set_velocity':
                velocity = data.get('velocity', {})
                await self.handle_set_velocity(velocity)
            elif command == 'add_goals':
                goals = data.get('goals', [])
                await self.handle_add_goals(goals)
            else:
                self.get_logger().warn(f'Unknown command: {command}')
                
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON received from client')
        except Exception as e:
            self.get_logger().error(f'Error handling client message: {str(e)}')
    
    async def handle_start_mission(self):
        """Handle start mission command."""
        if self.start_mission_client.service_is_ready():
            future = self.start_mission_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('Mission start requested')
    
    async def handle_stop_mission(self):
        """Handle stop mission command."""
        if self.stop_mission_client.service_is_ready():
            future = self.stop_mission_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('Mission stop requested')
    
    async def handle_pause_mission(self):
        """Handle pause mission command."""
        if self.pause_mission_client.service_is_ready():
            future = self.pause_mission_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info('Mission pause requested')
    
    async def handle_emergency_stop(self):
        """Handle emergency stop command."""
        # Send emergency stop command
        stop_msg = String()
        stop_msg.data = "emergency_stop"
        self.emergency_stop_pub.publish(stop_msg)
        
        # Stop robot immediately
        stop_vel = Twist()
        self.cmd_vel_pub.publish(stop_vel)
        
        self.get_logger().warn('Emergency stop activated')
    
    async def handle_set_velocity(self, velocity_data):
        """Handle manual velocity control."""
        vel_msg = Twist()
        vel_msg.linear.x = velocity_data.get('linear_x', 0.0)
        vel_msg.linear.y = velocity_data.get('linear_y', 0.0)
        vel_msg.angular.z = velocity_data.get('angular_z', 0.0)
        
        self.cmd_vel_pub.publish(vel_msg)
    
    async def handle_add_goals(self, goals_data):
        """Handle adding new goals to mission queue."""
        # This would typically call a service to add goals
        self.get_logger().info(f'Received {len(goals_data)} goals to add')
    
    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }
        
        self.current_velocity = {
            'linear_x': msg.twist.twist.linear.x,
            'linear_y': msg.twist.twist.linear.y,
            'angular_z': msg.twist.twist.angular.z
        }
    
    def mission_state_callback(self, msg):
        """Handle mission state updates."""
        self.mission_state = msg.data
    
    def mission_progress_callback(self, msg):
        """Handle mission progress updates."""
        try:
            progress_data = json.loads(msg.data)
            self.goal_state = progress_data.get('state', 'unknown')
        except json.JSONDecodeError:
            pass
    
    def broadcast_telemetry(self):
        """Broadcast telemetry to all connected clients."""
        if not self.connections:
            return
        
        telemetry = self.create_telemetry_data()
        message = json.dumps(telemetry)
        
        # Send to all connections asynchronously
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def send_to_all():
            if self.connections:
                await asyncio.gather(
                    *[ws.send(message) for ws in self.connections.copy()],
                    return_exceptions=True
                )
        
        loop.run_until_complete(send_to_all())
        loop.close()
    
    async def send_telemetry_to_client(self, websocket):
        """Send current telemetry to a specific client."""
        telemetry = self.create_telemetry_data()
        message = json.dumps(telemetry)
        await websocket.send(message)
    
    def create_telemetry_data(self):
        """Create telemetry data dictionary."""
        return {
            'timestamp': time.time(),
            'pose': self.current_pose,
            'velocity': self.current_velocity,
            'battery_level': self.battery_level,
            'mission_state': self.mission_state,
            'goal_state': self.goal_state,
            'connections': len(self.connections)
        }
    
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
    
    web_bridge = WebBridge()
    
    try:
        rclpy.spin(web_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        web_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

