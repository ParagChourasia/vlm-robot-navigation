"""
ROS 2 Node for Visual Question Answering using BLIP-2

This node subscribes to camera images and provides a service for answering
visual questions about the robot's environment.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque
import numpy as np
import logging
import sys

# Import the BLIP-2 model wrapper
from robot_vqa.blip2_model import BLIP2Model

# This will be updated once we build the interface package
try:
    from robot_vqa_interfaces.srv import AskQuestion
except ImportError:
    # Fallback for initial development
    AskQuestion = None


class VQANode(Node):
    """ROS 2 node for Visual Question Answering."""
    
    def __init__(self):
        super().__init__('vqa_node')
        
        # Setup logging
        self.logger = self.get_logger()
        self.logger.info("Initializing VQA Node...")
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('model_name', 'Salesforce/blip2-opt-2.7b')
        self.declare_parameter('device', 'auto')  # 'auto', 'cpu', or 'cuda'
        self.declare_parameter('image_buffer_size', 10)
        self.declare_parameter('service_name', 'ask_question')
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        model_name = self.get_parameter('model_name').value
        device = self.get_parameter('device').value
        buffer_size = self.get_parameter('image_buffer_size').value
        service_name = self.get_parameter('service_name').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Image buffer - stores recent images with timestamps
        self.image_buffer = deque(maxlen=buffer_size)
        self.latest_image = None
        self.latest_timestamp = None
        
        # Initialize BLIP-2 model
        device_str = None if device == 'auto' else device
        try:
            self.logger.info(f"Loading BLIP-2 model: {model_name}")
            self.vqa_model = BLIP2Model(model_name=model_name, device=device_str)
            self.logger.info("BLIP-2 model loaded successfully!")
        except Exception as e:
            self.logger.error(f"Failed to load model: {e}")
            self.logger.error("Node will exit.")
            sys.exit(1)
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        self.logger.info(f"Subscribed to camera topic: {camera_topic}")
        
        # Create VQA service
        if AskQuestion is not None:
            self.vqa_service = self.create_service(
                AskQuestion,
                service_name,
                self.ask_question_callback
            )
            self.logger.info(f"VQA service '{service_name}' ready")
        else:
            self.logger.warning("AskQuestion service interface not available yet")
        
        self.logger.info("VQA Node initialized successfully!")
    
    def image_callback(self, msg):
        """
        Callback for receiving camera images.
        
        Args:
            msg: sensor_msgs/Image message
        """
        try:
            # Convert ROS Image message to numpy array (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert BGR to RGB (BLIP-2 expects RGB)
            rgb_image = cv_image[:, :, ::-1]
            
            # Store in buffer with timestamp
            timestamp = msg.header.stamp
            self.image_buffer.append((timestamp, rgb_image))
            
            # Update latest image
            self.latest_image = rgb_image
            self.latest_timestamp = timestamp
            
        except Exception as e:
            self.logger.error(f"Error processing image: {e}")
    
    def ask_question_callback(self, request, response):
        """
        Service callback for answering visual questions.
        
        Args:
            request: AskQuestion service request
            response: AskQuestion service response
            
        Returns:
            Filled response object
        """
        question = request.question
        use_latest = request.use_latest_image
        
        self.logger.info(f"Received question: '{question}'")
        
        # Check if we have any images
        if self.latest_image is None:
            response.success = False
            response.error_message = "No images received yet"
            response.answer = ""
            response.confidence = 0.0
            self.logger.warning("No images available")
            return response
        
        # Select image (for now, always use latest)
        # TODO: Implement timestamp-based image selection
        image = self.latest_image
        
        try:
            # Check if question is empty (generate caption instead)
            if not question or question.strip() == "":
                self.logger.info("Empty question - generating caption")
                answer, confidence = self.vqa_model.generate_caption(image)
            else:
                # Answer the question
                answer, confidence = self.vqa_model.answer_question(image, question)
            
            # Fill response
            response.success = True
            response.answer = answer
            response.confidence = confidence
            response.error_message = ""
            
            self.logger.info(f"Answer: '{answer}' (confidence: {confidence:.2f})")
            
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            response.answer = ""
            response.confidence = 0.0
            self.logger.error(f"Error during inference: {e}")
        
        return response


def main(args=None):
    """Main entry point for the VQA node."""
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='[%(levelname)s] [%(name)s]: %(message)s'
    )
    
    rclpy.init(args=args)
    
    try:
        node = VQANode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f"Fatal error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
