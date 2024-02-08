# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
import time

# Defining the SimplePublisher class which inherits from Node
class DataCollection(Node):

    def __init__(self):
        super().__init__('data_collection')  # Initialize the node with the name 'simple_publisher'
        # Create a subscriber to get user target
        # self.command_target_subscriber = self.create_subscription(
        #     Float32MultiArray,
        #     '/control_target',
        #     self.get_command_target,
        #     1)
        # self.command_target_subscriber
        # self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos_rel',10)

        self.publisher_ = self.create_publisher(Float32MultiArray, '/joint_vel', 1)
        timer_period = 1  # Setting the timer period to 1 second
        # Create a timer that calls the timer_callback method every 1 second
        self.timer = self.create_timer(timer_period, self.publish)
        # self.publish()
    def publish(self):
        # for i in range(50):
        #     msg = Float32MultiArray()  # Creating a String message object
        #     msg.data = [10.0*i]  # Setting the message data
        #     self.publisher_.publish(msg)  # Publishing the message
        #     # Logging the published message to the console
        #     self.get_logger().info('Publishing: "%s"' % msg.data)
            
            # msg = Float32MultiArray()  # Creating a String message object
            # msg.data = [float(0)]  # Setting the message data
            # self.publisher_.publish(msg)  # Publishing the message
            # # Logging the published message to the console
            # self.get_logger().info('Publishing: "%s"' % msg.data)


        for i in range(-10, 11):
            msg = Float32MultiArray()  # Creating a String message object
            msg.data = [float(i)]  # Setting the message data
            self.publisher_.publish(msg)  # Publishing the message
            # Logging the published message to the console
            self.get_logger().info('Publishing: "%s"' % msg.data)
            time.sleep(5)
# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    simple_publisher = DataCollection()  # Create an instance of the SimplePublisher
    # simple_publisher.publish()
    try:
        rclpy.spin_once(simple_publisher)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    simple_publisher.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
