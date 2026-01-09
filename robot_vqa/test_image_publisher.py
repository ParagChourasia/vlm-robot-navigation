#!/usr/bin/env python3
"""
Simple image publisher for testing VQA

Publishes a static image to a ROS 2 topic for testing the VQA system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import os


class ImagePublisher(Node):
    """Publishes static images to a topic."""
    
    def __init__(self, image_path, topic='/camera/image_raw', rate=1.0):
        super().__init__('test_image_publisher')
        
        self.bridge = CvBridge()
        
        # Load image
        if not os.path.exists(image_path):
            self.get_logger().error(f"Image not found: {image_path}")
            sys.exit(1)
        
        self.image = cv2.imread(image_path)
        if self.image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            sys.exit(1)
        
        self.get_logger().info(f"Loaded image: {image_path}")
        self.get_logger().info(f"Image shape: {self.image.shape}")
        
        # Create publisher
        self.publisher = self.create_publisher(Image, topic, 10)
        
        # Create timer
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Publishing to '{topic}' at {rate} Hz")
    
    def timer_callback(self):
        """Publish the image."""
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.publisher.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("\nUsage:")
        print("  python3 test_image_publisher.py <image_path> [topic] [rate_hz]")
        print("\nExample:")
        print("  python3 test_image_publisher.py test_images/kitchen.jpg")
        print("  python3 test_image_publisher.py test_images/office.jpg /camera/image_raw 2.0")
        print()
        sys.exit(1)
    
    image_path = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else '/camera/image_raw'
    rate = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    
    try:
        node = ImagePublisher(image_path, topic, rate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
