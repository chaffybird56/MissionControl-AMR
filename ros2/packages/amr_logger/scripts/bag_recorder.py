#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import time
import threading
from typing import Optional


class BagRecorder(Node):
    """
    Automatic rosbag2 recorder that starts/stops recording based on mission state.
    Records relevant topics during mission execution.
    """
    
    def __init__(self):
        super().__init__('bag_recorder')
        
        # Parameters
        self.declare_parameter('bag_directory', '/bags')
        self.declare_parameter('topics_to_record', [
            '/odom', '/scan', '/camera/image_raw', '/cmd_vel',
            '/tf', '/tf_static', '/map', '/mission/state',
            '/mission/progress', '/navigate_to_pose/_action/status'
        ])
        self.declare_parameter('bag_prefix', 'amr_mission')
        self.declare_parameter('compression_mode', 'none')
        
        # Get parameters
        self.bag_directory = self.get_parameter('bag_directory').value
        self.topics_to_record = self.get_parameter('topics_to_record').value
        self.bag_prefix = self.get_parameter('bag_prefix').value
        self.compression_mode = self.get_parameter('compression_mode').value
        
        # Recording state
        self.is_recording = False
        self.current_bag_process: Optional[subprocess.Popen] = None
        self.current_bag_name = None
        self.recording_lock = threading.Lock()
        
        # Create bag directory if it doesn't exist
        os.makedirs(self.bag_directory, exist_ok=True)
        
        # Subscribers
        self.mission_state_sub = self.create_subscription(
            String,
            '/mission/state',
            self.mission_state_callback,
            10
        )
        
        self.get_logger().info(f'Bag recorder initialized, recording to: {self.bag_directory}')
        self.get_logger().info(f'Topics to record: {self.topics_to_record}')
    
    def mission_state_callback(self, msg):
        """Handle mission state changes to start/stop recording."""
        state = msg.data
        
        with self.recording_lock:
            if state == 'running' and not self.is_recording:
                self.start_recording()
            elif state in ['succeeded', 'failed', 'cancelled'] and self.is_recording:
                self.stop_recording()
    
    def start_recording(self):
        """Start rosbag2 recording."""
        if self.is_recording:
            self.get_logger().warn('Already recording, ignoring start request')
            return
        
        # Generate bag name with timestamp
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        self.current_bag_name = f"{self.bag_prefix}_{timestamp}"
        bag_path = os.path.join(self.bag_directory, self.current_bag_name)
        
        # Build rosbag2 record command
        cmd = ['ros2', 'bag', 'record', '-o', bag_path]
        
        # Add compression if specified
        if self.compression_mode != 'none':
            cmd.extend(['--compression-mode', self.compression_mode])
        
        # Add topics to record
        cmd.extend(self.topics_to_record)
        
        try:
            # Start recording process
            self.current_bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            self.is_recording = True
            self.get_logger().info(f'Started recording bag: {self.current_bag_name}')
            self.get_logger().info(f'Recording topics: {", ".join(self.topics_to_record)}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {str(e)}')
            self.is_recording = False
            self.current_bag_process = None
    
    def stop_recording(self):
        """Stop rosbag2 recording."""
        if not self.is_recording:
            self.get_logger().warn('Not recording, ignoring stop request')
            return
        
        if self.current_bag_process is None:
            self.get_logger().warn('No recording process to stop')
            self.is_recording = False
            return
        
        try:
            # Send SIGINT to the process group
            os.killpg(os.getpgid(self.current_bag_process.pid), 2)
            
            # Wait for process to terminate
            try:
                self.current_bag_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop gracefully
                os.killpg(os.getpgid(self.current_bag_process.pid), 9)
                self.current_bag_process.wait()
            
            # Get output for logging
            stdout, stderr = self.current_bag_process.communicate()
            
            self.is_recording = False
            self.get_logger().info(f'Stopped recording bag: {self.current_bag_name}')
            
            # Log any errors
            if stderr:
                self.get_logger().warn(f'Recording errors: {stderr.decode()}')
            
        except Exception as e:
            self.get_logger().error(f'Error stopping recording: {str(e)}')
            self.is_recording = False
        finally:
            self.current_bag_process = None
            self.current_bag_name = None
    
    def get_recording_status(self):
        """Get current recording status."""
        with self.recording_lock:
            return {
                'is_recording': self.is_recording,
                'current_bag': self.current_bag_name,
                'bag_directory': self.bag_directory
            }
    
    def cleanup(self):
        """Clean up resources on shutdown."""
        if self.is_recording:
            self.get_logger().info('Shutting down, stopping recording...')
            self.stop_recording()


def main(args=None):
    rclpy.init(args=args)
    
    bag_recorder = BagRecorder()
    
    try:
        rclpy.spin(bag_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        bag_recorder.cleanup()
        bag_recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

