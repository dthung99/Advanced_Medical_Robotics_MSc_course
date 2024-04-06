# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Import the String message type from standard ROS2 message library
import matplotlib.pyplot as plt
import time
import numpy as np
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import subprocess
from haptic.module_my_math import Kinematics
import matplotlib.colors as mcolors
import pickle
import threading

# Defining the SimpleSubscriber class which inherits from Node
class SavePNG(Node):

    def __init__(self):
        super().__init__('simple_subscriber')  # Initialize the node with the name 'simple_subscriber'
        self.environment_name = "/home/pi/ros2_ws/src/haptic/haptic/Environment.pk1"
        # Load the environment
        with open(self.environment_name, 'rb') as file:
            self.environment = pickle.load(file)
        self.image_array = np.array(self.environment)[:,:,0]

        # self.PNG_export_name = "/home/pi/ros2_ws/src/haptic/haptic/PNG.jpeg"
        custom_colors = ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF', '#FFA500', '#800080', '#008080', '#800000']
        self.custom_cmap = mcolors.ListedColormap(custom_colors)

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 300)  # Replace xmin and xmax with your desired limits
        self.ax.set_ylim(0, 600)  # Replace ymin and ymax with your desired limits
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()
        self.ax.axis("off")

        self.arm_1_plot, = self.ax.plot("r-") 
        self.arm_2_plot, = self.ax.plot("g-") 
        self.arm_3_plot, = self.ax.plot("b-")
        self.ax.imshow(self.image_array.astype(np.uint8), cmap=self.custom_cmap)
        # Declare robot handler
        self.robot_state = Kinematics([0,0,0])
        self.get_current_position = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.save_PNG,
            1)
        self.get_current_position  # Dummy expression to avoid unused variable warning
    def save_PNG(self, msg):
        start_time = time.perf_counter()
        self.robot_state.Update_Data(-np.array(msg.data)[0:3])
        self.arm_1_plot.set_data([0, self.robot_state.arm_1_position[0]], [300, 300-self.robot_state.arm_1_position[1]])
        self.arm_2_plot.set_data([self.robot_state.arm_1_position[0], self.robot_state.arm_2_position[0]], [300-self.robot_state.arm_1_position[1], 300-self.robot_state.arm_2_position[1]])
        self.arm_3_plot.set_data([self.robot_state.arm_2_position[0], self.robot_state.arm_3_position[0]], [300-self.robot_state.arm_2_position[1], 300-self.robot_state.arm_3_position[1]])
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        plt.pause(0.01)
        end_time = time.perf_counter()
        print(f"Time need: {end_time - start_time}")

# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    simple_subscriber = SavePNG()  # Create an instance of the SimpleSubscriber
    print("Node started")
    try:
        rclpy.spin(simple_subscriber)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass
    simple_subscriber.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()







