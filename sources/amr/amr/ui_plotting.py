# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from amr.module_kinematics import Forward_Kinematics
from std_msgs.msg import Float32MultiArray
import threading
import time
import numpy as np
import matplotlib.pyplot as plt 

class UI_plotting(Node):

    def __init__(self):
        super().__init__('ui_plotting')
        # Create a subscription
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.listener_callback,
            10)
        self.subscription
        # Create a kinematics object
        self.robot_kinematics = Forward_Kinematics([0, 0, 0])
        # Create a subscriber
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/UI_message',10)
    
    def listener_callback(self, msg):
        print(2)

        

# The main function which serves as the entry point for the program
def main(args=None):
    # global main_node
    # # Create a thread
    # def spinning_node():
    #     global main_node
    #     rclpy.init(args=None)
    #     main_node = UI_plotting()
    #     print("Node started")
    #     try:
    #         rclpy.spin(main_node)  # Keep the node alive and listening for messages
    #     except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
    #         pass
    #     finally:
    #         print("Finished spinning")
    #         main_node.destroy_node()  # Properly destroy the node
    #         rclpy.shutdown()  # Shutdown the ROS2 Python client library
    # spinning_thread = threading.Thread(target=spinning_node, args = ())
    # spinning_thread.start()
    # time.sleep(1)
    # print(main_node.robot_kinematics.arm_1)
    # plt.show()
    # Data for the plot
    x = [1, 2, 3, 4, 5]
    y = [2, 4, 6, 8, 10]

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Plot the data
    ax.plot(x, y)

    # Set labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('Simple Plot')

    # Show the plot
    plt.show()

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
