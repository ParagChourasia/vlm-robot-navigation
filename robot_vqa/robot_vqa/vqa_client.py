"""
CLI Client for Visual Question Answering Service

Simple command-line client to interact with the VQA service.
"""

import rclpy
from rclpy.node import Node
import sys

try:
    from robot_vqa_interfaces.srv import AskQuestion
except ImportError:
    print("ERROR: robot_vqa_interfaces not built yet")
    print("Please build the workspace first: colcon build --packages-select robot_vqa_interfaces")
    sys.exit(1)


class VQAClient(Node):
    """Simple client for VQA service."""
    
    def __init__(self):
        super().__init__('vqa_client')
        self.client = self.create_client(AskQuestion, 'ask_question')
        
        # Wait for service to be available
        self.get_logger().info("Waiting for VQA service...")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        
        self.get_logger().info("VQA service is ready!")
    
    def ask_question(self, question):
        """
        Ask a question to the VQA service.
        
        Args:
            question: Natural language question string
            
        Returns:
            Response from the service
        """
        request = AskQuestion.Request()
        request.question = question
        request.use_latest_image = True
        
        self.get_logger().info(f"Asking: '{question}'")
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    """Main entry point for the CLI client."""
    rclpy.init(args=args)
    
    # Get question from command line arguments
    if len(sys.argv) < 2:
        print("\nUsage:")
        print("  ros2 run robot_vqa vqa_client \"Your question here\"")
        print("\nExamples:")
        print("  ros2 run robot_vqa vqa_client \"What do you see?\"")
        print("  ros2 run robot_vqa vqa_client \"How many objects are there?\"")
        print("  ros2 run robot_vqa vqa_client \"\"  # Empty question generates caption")
        print()
        sys.exit(1)
    
    question = sys.argv[1]
    
    try:
        client = VQAClient()
        response = client.ask_question(question)
        
        print("\n" + "="*60)
        if response.success:
            print(f"Question: {question if question else '(caption mode)'}")
            print(f"Answer: {response.answer}")
            print(f"Confidence: {response.confidence:.2f}")
        else:
            print(f"Error: {response.error_message}")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
