#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose


class FiducialDetector(Node):
    """
    Simple fiducial detector using OpenCV ArUco markers.
    Detects ArUco markers in camera images and publishes their poses.
    """
    
    def __init__(self):
        super().__init__('fiducial_detector')
        
        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('landmarks_topic', '/landmarks')
        self.declare_parameter('detected_goal_topic', '/detected_goal')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.2)  # meters
        self.declare_parameter('camera_matrix', [320.0, 0.0, 160.0, 0.0, 320.0, 120.0, 0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        landmarks_topic = self.get_parameter('landmarks_topic').value
        detected_goal_topic = self.get_parameter('detected_goal_topic').value
        aruco_dict_name = self.get_parameter('aruco_dictionary').value
        self.marker_size = self.get_parameter('marker_size').value
        
        # Camera calibration parameters
        camera_matrix = np.array(self.get_parameter('camera_matrix').value).reshape(3, 3)
        dist_coeffs = np.array(self.get_parameter('dist_coeffs').value)
        
        # Initialize ArUco detector
        aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, aruco_dict_name))
        parameters = cv2.aruco.DetectorParameters_create()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        
        # Camera parameters for pose estimation
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        # CV bridge
        self.bridge = CvBridge()
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.landmarks_pub = self.create_publisher(PoseStamped, landmarks_topic, 10)
        self.detected_goal_pub = self.create_publisher(PoseStamped, detected_goal_topic, 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Marker ID to goal pose mapping (for demo purposes)
        self.marker_goals = {
            0: [2.0, 2.0, 0.0],  # Marker 0 -> goal at (2, 2)
            1: [-2.0, 1.0, 0.0], # Marker 1 -> goal at (-2, 1)
            2: [0.0, -2.0, 0.0], # Marker 2 -> goal at (0, -2)
        }
        
        self.get_logger().info('Fiducial detector initialized')
    
    def image_callback(self, msg):
        """Process incoming camera image and detect fiducials."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect ArUco markers
            marker_corners, marker_ids, _ = self.detector.detectMarkers(cv_image)
            
            if marker_ids is not None:
                # Estimate pose for each detected marker
                for i, marker_id in enumerate(marker_ids.flatten()):
                    # Get marker corners
                    corners = marker_corners[i][0]
                    
                    # Estimate pose
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners], self.marker_size, self.camera_matrix, self.dist_coeffs
                    )
                    
                    # Convert rotation vector to rotation matrix
                    rotation_matrix, _ = cv2.Rodrigues(rvec[0])
                    
                    # Create pose message
                    pose_msg = PoseStamped()
                    pose_msg.header = msg.header
                    pose_msg.header.frame_id = "camera_link"
                    
                    # Position
                    pose_msg.pose.position.x = float(tvec[0][0])
                    pose_msg.pose.position.y = float(tvec[0][1])
                    pose_msg.pose.position.z = float(tvec[0][2])
                    
                    # Convert rotation matrix to quaternion
                    quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
                    pose_msg.pose.orientation.x = quaternion[0]
                    pose_msg.pose.orientation.y = quaternion[1]
                    pose_msg.pose.orientation.z = quaternion[2]
                    pose_msg.pose.orientation.w = quaternion[3]
                    
                    # Publish landmark pose
                    self.landmarks_pub.publish(pose_msg)
                    
                    # Check if this marker corresponds to a goal
                    if marker_id in self.marker_goals:
                        goal_pos = self.marker_goals[marker_id]
                        goal_pose = PoseStamped()
                        goal_pose.header = msg.header
                        goal_pose.header.frame_id = "map"
                        goal_pose.pose.position.x = goal_pos[0]
                        goal_pose.pose.position.y = goal_pos[1]
                        goal_pose.pose.position.z = goal_pos[2]
                        goal_pose.pose.orientation.w = 1.0
                        
                        self.detected_goal_pub.publish(goal_pose)
                        self.get_logger().info(f'Detected goal marker {marker_id} at {goal_pos}')
                    
                    self.get_logger().debug(f'Detected marker {marker_id} at position {tvec[0]}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion."""
        trace = np.trace(R)
        
        if trace > 0:
            s = math.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    
    fiducial_detector = FiducialDetector()
    
    try:
        rclpy.spin(fiducial_detector)
    except KeyboardInterrupt:
        pass
    finally:
        fiducial_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

