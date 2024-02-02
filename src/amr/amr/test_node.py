# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
import scipy

# Defining the SimplePublisher class which inherits from Node
class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')  # Initialize the node with the name 'simple_publisher'

        self.subscriber_1 = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.print_1,
            10)
        self.subscriber_1  # Dummy expression to avoid unused variable warning

        self.subscriber_2 = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.print_2,
            10)
        self.subscriber_2  # Dummy expression to avoid unused variable warning
    def print_1(self, msg):
        print(1)
    def print_2(self, msg):
        print(2)
# The main function which serves as the entry point for the program
def main(args=None):
    print(2)

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
