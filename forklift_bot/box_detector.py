#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class BoxDetector(Node):
    # Detects colored boxes in camera feed using computer vision.
    
    def __init__(self):
        super().__init__('box_detector')
        
        self.declare_parameter('min_area', 500)
        self.declare_parameter('debug_view', False)
        
        self.min_area = self.get_parameter('min_area').value
        self.debug_view = self.get_parameter('debug_view').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_pub = self.create_publisher(Bool, 'box_detected', 10)
        self.box_position_pub = self.create_publisher(Point, 'box_position', 10)
        
        if self.debug_view:
            self.debug_image_pub = self.create_publisher(Image, 'detection_debug', 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        
        self.get_logger().info(f'Box Detector initialized - Looking for any colored boxes (red, blue, green)')
    
    def get_all_color_ranges(self):
        # Get HSV color ranges for red, blue, and green boxes
        return {
            'red': [
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'blue': [
                (np.array([100, 100, 100]), np.array([130, 255, 255]))
            ],
            'green': [
                (np.array([40, 50, 50]), np.array([80, 255, 255]))
            ]
        }
    
    def image_callback(self, msg):
        # Process camera image to detect boxes of any color.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create combined mask for all colors (red, blue, green)
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        all_color_ranges = self.get_all_color_ranges()
        
        for color_name, color_ranges in all_color_ranges.items():
            for lower, upper in color_ranges:
                color_mask = cv2.inRange(hsv, lower, upper)
                combined_mask = cv2.bitwise_or(combined_mask, color_mask)
        
        # Clean up mask with morphological operations
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected = Bool()
        detected.data = False
        
        if contours:
            # Find largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_area:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Normalize coordinates (-1 to 1, where 0 is center)
                    height, width = frame.shape[:2]
                    normalized_x = (cx - width / 2) / (width / 2)
                    normalized_y = (cy - height / 2) / (height / 2)
                    
                    # Estimate distance based on box size
                    estimated_distance = 5000.0 / np.sqrt(area)
                    
                    # Publish detection
                    detected.data = True
                    
                    position = Point()
                    position.x = normalized_x
                    position.y = normalized_y
                    position.z = estimated_distance
                    
                    self.box_position_pub.publish(position)
                    
                    # Debug visualization
                    if self.debug_view:
                        debug_frame = frame.copy()
                        cv2.drawContours(debug_frame, [largest_contour], -1, (0, 255, 0), 3)
                        cv2.circle(debug_frame, (cx, cy), 10, (255, 0, 0), -1)
                        cv2.putText(debug_frame, f'Area: {int(area)}', (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(debug_frame, f'Dist: {estimated_distance:.2f}m', (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, "bgr8")
                        self.debug_image_pub.publish(debug_msg)
        
        self.detection_pub.publish(detected)


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

