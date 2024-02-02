# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from amr.module_kinematics import Forward_Kinematics
from std_msgs.msg import Float32MultiArray
import threading

# Defining the SimpleSubscriber class which inherits from Node
class UI_plotting(Node):

    def __init__(self):
        super().__init__('UI_plotting')  # Initialize the node with the name 'simple_subscriber'
        # Create a subscription object that listens to messages of type String
        # on the topic 'topic'. The 'listener_callback' function is called
        # when a new message is received. '10' is the queue size.
        self.subscription = self.create_subscription(
            String,
            '/joint_state',
            self.listener_callback,
            10)
        self.subscription  # Dummy expression to avoid unused variable warning
        self.robot_kinematics = Forward_Kinematics([0, 0, 0])
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/UI_message',10)

    def listener_callback(self, msg):
        # Callback function that is invoked when a new message is received
        self.get_logger().info('I heard: "%s"' % msg.data)  # Log the received message

# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    simple_subscriber = UI_plotting()  # Create an instance of the SimpleSubscriber

    try:
        rclpy.spin(simple_subscriber)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    simple_subscriber.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
