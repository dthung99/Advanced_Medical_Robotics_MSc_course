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
        # Plotting variable
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 300)  # Replace xmin and xmax with your desired limits
        self.ax.set_ylim(0, 600)  # Replace ymin and ymax with your desired limits
        self.ax.set_aspect('equal')
        self.ax.invert_yaxis()
        self.ax.axis("off")

        angles = np.linspace(-np.pi/2, np.pi/2, 100)
        # Generate the x and y coordinates of the points on the circle
        x = np.cos(angles)*300
        y = np.sin(angles)*300+300
        self.ax.plot(x, y, "k--", linewidth=2)

        self.arm_1_plot, = self.ax.plot("r-") 
        self.arm_2_plot, = self.ax.plot("g-") 
        self.arm_3_plot, = self.ax.plot("b-")

        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        # Declare robot handler
        self.robot_state = Kinematics([0,0,0])
        self.get_current_position = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.save_PNG,
            1)
        self.get_current_position  # Dummy expression to avoid unused variable warning
        # Create a publisher
        self.sendtarget = self.create_publisher(Float32MultiArray,'/final_target',1)
        self.msg = Float32MultiArray()  # Creating a String message object

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

    def onclick(self, event):
        if event.inaxes:
            # Extract the x and y coordinates of the click event
            x = event.xdata
            y = 300 - event.ydata
            if (x**2 + y**2) > 90000:
                print("Too far away")
            elif (x <= 0):
                print("Noop, I'm not going there")
            else:
                self.msg.data = [x,y,0.0] # Setting the message data
                self.sendtarget.publish(self.msg)

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







