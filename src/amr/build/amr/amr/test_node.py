# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
import numpy as np

# Defining the SimplePublisher class which inherits from Node
class DataCollection(Node):

    def __init__(self):
        super().__init__('data_collection')  # Initialize the node with the name 'simple_publisher'
        # Create a subscriber to get user target
        self.command_target_subscriber = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.get_command_target,
            1)
        self.command_target_subscriber
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_cur', 1)
    def get_command_target(self, msg):
        print(np.array(msg.data)[0])
        msg = Float32MultiArray()  # Creating a String message object
        msg.data = [50.0]  # Setting the message data
        self.subscriber_publisher.publish(msg)  # Publishing the message
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    simple_publisher = DataCollection()  # Create an instance of the SimplePublisher
    # simple_publisher.publish()
    print("Node started")
    try:
        rclpy.spin(simple_publisher)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    simple_publisher.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
