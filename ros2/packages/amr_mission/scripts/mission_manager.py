#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
import json
import time
from enum import Enum


class MissionState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELLED = "cancelled"


class MissionManager(Node):
    """
    Mission manager that handles a queue of navigation goals.
    Manages mission state and coordinates with Nav2 action server.
    """
    
    def __init__(self):
        super().__init__('mission_manager')
        
        # Parameters
        self.declare_parameter('mission_timeout', 300.0)  # 5 minutes default
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('max_retries', 3)
        
        # Get parameters
        self.mission_timeout = self.get_parameter('mission_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_retries = self.get_parameter('max_retries').value
        
        # Mission state
        self.current_state = MissionState.IDLE
        self.mission_queue = []
        self.current_goal_index = 0
        self.mission_start_time = None
        self.retry_count = 0
        
        # Metrics
        self.total_goals_completed = 0
        self.total_replans = 0
        self.total_distance_traveled = 0.0
        self.mission_history = []
        
        # Action client for navigation
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.mission_state_pub = self.create_publisher(String, '/mission/state', 10)
        self.mission_progress_pub = self.create_publisher(String, '/mission/progress', 10)
        self.metrics_pub = self.create_publisher(String, '/mission/metrics', 10)
        
        # Services
        self.start_mission_srv = self.create_service(
            Empty, '/mission/start', self.start_mission_callback
        )
        self.stop_mission_srv = self.create_service(
            Empty, '/mission/stop', self.stop_mission_callback
        )
        self.pause_mission_srv = self.create_service(
            Empty, '/mission/pause', self.pause_mission_callback
        )
        self.resume_mission_srv = self.create_service(
            Empty, '/mission/resume', self.resume_mission_callback
        )
        self.add_goals_srv = self.create_service(
            Empty, '/mission/add_goals', self.add_goals_callback
        )
        
        # Timer for publishing status
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Wait for action server
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.nav_action_client.wait_for_server()
        self.get_logger().info('Mission manager initialized')
    
    def set_mission_queue(self, goals):
        """Set the mission queue with a list of goal poses."""
        self.mission_queue = goals
        self.current_goal_index = 0
        self.retry_count = 0
        self.get_logger().info(f'Mission queue set with {len(goals)} goals')
    
    def start_mission_callback(self, request, response):
        """Service callback to start the current mission."""
        if not self.mission_queue:
            self.get_logger().warn('No goals in mission queue')
            return response
        
        if self.current_state == MissionState.RUNNING:
            self.get_logger().warn('Mission already running')
            return response
        
        self.current_state = MissionState.RUNNING
        self.mission_start_time = time.time()
        self.current_goal_index = 0
        self.retry_count = 0
        
        # Start executing the first goal
        self.execute_next_goal()
        
        self.get_logger().info('Mission started')
        return response
    
    def stop_mission_callback(self, request, response):
        """Service callback to stop the current mission."""
        if self.current_state == MissionState.RUNNING:
            # Cancel current navigation goal
            if self.nav_action_client.server_is_ready():
                future = self.nav_action_client.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future)
            
            self.current_state = MissionState.CANCELLED
            self.get_logger().info('Mission stopped')
        
        return response
    
    def pause_mission_callback(self, request, response):
        """Service callback to pause the current mission."""
        if self.current_state == MissionState.RUNNING:
            # Cancel current navigation goal
            if self.nav_action_client.server_is_ready():
                future = self.nav_action_client.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future)
            
            self.current_state = MissionState.PAUSED
            self.get_logger().info('Mission paused')
        
        return response
    
    def resume_mission_callback(self, request, response):
        """Service callback to resume the paused mission."""
        if self.current_state == MissionState.PAUSED:
            self.current_state = MissionState.RUNNING
            self.execute_next_goal()
            self.get_logger().info('Mission resumed')
        
        return response
    
    def add_goals_callback(self, request, response):
        """Service callback to add goals to the mission queue."""
        # This would typically accept goal poses in the request
        # For now, just return success
        return response
    
    def execute_next_goal(self):
        """Execute the next goal in the mission queue."""
        if not self.mission_queue or self.current_goal_index >= len(self.mission_queue):
            self.complete_mission()
            return
        
        goal_pose = self.mission_queue[self.current_goal_index]
        self.get_logger().info(f'Executing goal {self.current_goal_index + 1}/{len(self.mission_queue)}: {goal_pose}')
        
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        # Send goal
        send_goal_future = self.nav_action_client.send_goal_async(
            nav_goal, feedback_callback=self.navigation_feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            self.handle_goal_failure()
            return
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        if result:
            self.handle_goal_success()
        else:
            self.handle_goal_failure()
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        self.total_replans += 1
        
        # Update distance traveled (approximate)
        if hasattr(feedback, 'distance_remaining'):
            # This would be calculated from odometry in a real implementation
            pass
    
    def handle_goal_success(self):
        """Handle successful goal completion."""
        self.total_goals_completed += 1
        self.current_goal_index += 1
        self.retry_count = 0
        
        self.get_logger().info(f'Goal {self.current_goal_index} completed successfully')
        
        # Execute next goal
        if self.current_state == MissionState.RUNNING:
            self.execute_next_goal()
    
    def handle_goal_failure(self):
        """Handle goal failure with retry logic."""
        self.retry_count += 1
        
        if self.retry_count < self.max_retries:
            self.get_logger().warn(f'Goal failed, retrying ({self.retry_count}/{self.max_retries})')
            # Retry current goal
            if self.current_state == MissionState.RUNNING:
                self.execute_next_goal()
        else:
            self.get_logger().error(f'Goal failed after {self.max_retries} retries')
            self.current_state = MissionState.FAILED
            self.current_goal_index += 1
            self.retry_count = 0
    
    def complete_mission(self):
        """Complete the mission and update metrics."""
        self.current_state = MissionState.SUCCEEDED
        mission_duration = time.time() - self.mission_start_time if self.mission_start_time else 0
        
        # Record mission in history
        mission_record = {
            'timestamp': time.time(),
            'duration': mission_duration,
            'goals_completed': self.total_goals_completed,
            'replans': self.total_replans,
            'success_rate': self.total_goals_completed / len(self.mission_queue) if self.mission_queue else 0
        }
        self.mission_history.append(mission_record)
        
        self.get_logger().info(f'Mission completed successfully in {mission_duration:.2f}s')
        self.get_logger().info(f'Goals completed: {self.total_goals_completed}/{len(self.mission_queue)}')
        self.get_logger().info(f'Total replans: {self.total_replans}')
    
    def publish_status(self):
        """Publish current mission status."""
        # Publish state
        state_msg = String()
        state_msg.data = self.current_state.value
        self.mission_state_pub.publish(state_msg)
        
        # Publish progress
        progress_data = {
            'current_goal': self.current_goal_index,
            'total_goals': len(self.mission_queue),
            'state': self.current_state.value,
            'retry_count': self.retry_count
        }
        progress_msg = String()
        progress_msg.data = json.dumps(progress_data)
        self.mission_progress_pub.publish(progress_msg)
        
        # Publish metrics
        metrics_data = {
            'goals_completed': self.total_goals_completed,
            'total_replans': self.total_replans,
            'distance_traveled': self.total_distance_traveled,
            'success_rate': self.total_goals_completed / len(self.mission_queue) if self.mission_queue else 0
        }
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics_data)
        self.metrics_pub.publish(metrics_msg)


def main(args=None):
    rclpy.init(args=args)
    
    mission_manager = MissionManager()
    
    # Demo mission queue
    demo_goals = []
    for i in range(3):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(i * 2.0 - 2.0)
        goal.pose.position.y = float(i * 1.0)
        goal.pose.orientation.w = 1.0
        demo_goals.append(goal)
    
    mission_manager.set_mission_queue(demo_goals)
    
    try:
        rclpy.spin(mission_manager)
    except KeyboardInterrupt:
        pass
    finally:
        mission_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

