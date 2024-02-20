# # Importing necessary libraries from ROS2 Python client library
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
# from std_msgs.msg import Float32MultiArray
# import numpy as np

# # Defining the SimplePublisher class which inherits from Node
# class DataCollection(Node):

#     def __init__(self):
#         super().__init__('data_collection')  # Initialize the node with the name 'simple_publisher'
#         # Create a subscriber to get user target
#         self.command_target_subscriber = self.create_subscription(
#             Float32MultiArray,
#             '/joint_state',
#             self.get_command_target,
#             1)
#         self.command_target_subscriber
#         self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/micro_control_target', 1)
#     def get_command_target(self, msg):
#         print(np.array(msg.data)[0])
#         msg = Float32MultiArray()  # Creating a String message object
#         msg.data = [5.0, 5.0, 5.0]  # Setting the message data

#         self.subscriber_publisher.publish(msg)  # Publishing the message
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     simple_publisher = DataCollection()  # Create an instance of the SimplePublisher
#     # simple_publisher.publish()
#     print("Node started")
#     try:
#         rclpy.spin(simple_publisher)  # Keep the node alive and listening for messages
#     except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
#         pass

#     simple_publisher.destroy_node()  # Properly destroy the node
#     rclpy.shutdown()  # Shutdown the ROS2 Python client library

# # This condition checks if the script is executed directly (not imported)
# if __name__ == '__main__':
#     main()  # Execute the main function

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import numpy as np

# class UI_send_target(Node):
#     def __init__(self):
#         super().__init__('ui_send_target')
#         # Create a publisher
#         self.sendtarget = self.create_publisher(Float32MultiArray,'/micro_control_target',1)
        
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     UI_send_target_node = UI_send_target()  # Create an instance of the SimplePublisher
#     print("Node started")
#     try:
#         while True:
#             user_input = []
#             for i in range(3):
#                 user_input.append(float(input("Enter target: ")))
#             # user_input.append(0.0)
#             msg = Float32MultiArray()  # Creating a String message object
#             msg.data = user_input # Setting the message data
#             UI_send_target_node.sendtarget.publish(msg)
#             print(f"Target sent: {user_input}")
#     except KeyboardInterrupt:
#         pass
#     finally:
#         print()
#         print()
#         print("Thank you")
#         print()
#         print()
#         UI_send_target_node.destroy_node()  # Properly destroy the node
#         rclpy.shutdown()  # Shutdown the ROS2 Python client library
# if __name__ == '__main__':
#     main()  # Execute the main function




########################################## Data collection
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
        self.publisher_reset = self.create_publisher(Float32MultiArray, '/joint_pos', 1)
        timer_period = 1  # Setting the timer period to 1 second
        # Create a timer that calls the timer_callback method every 1 second
        self.timer = self.create_timer(timer_period, self.publish)
        # self.publish()
    def publish(self):
        # print(2)
        # msg = Float32MultiArray()  # Creating a String message object
        # msg.data = [0.0, 0.0, 0.0]  # Setting the message data
        # self.publisher_reset.publish(msg)
        sign = 1
        for i in range(-10, 11):            
            msg = Float32MultiArray()  # Creating a String message object
            j = i*sign
            sign = -1*sign
            msg.data = [float(j), -1*float(j), float(j)]  # Setting the message data
            self.publisher_.publish(msg)  # Publishing the message
            # Logging the published message to the console
            self.get_logger().info('Publishing: "%s"' % msg.data)
            time.sleep(0.5)
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
