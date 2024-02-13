# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

# Defining the SimplePublisher class which inherits from Node
class PIDAngleControl(Node):
    def __init__(self):
        super().__init__('pid_angle_control')  # Initialize the node with the name 'simple_publisher'
        # Declare parameters
        self.kp = 1
        self.ki = 1
        self.kd = 0.5
        self.start_moving = False
        self.old_error = 0
        # Create global msg
        self.current_msg = Float32MultiArray()  # Creating a String message object
        # Create a subscriber to get user target
        self.micro_target_subscriber = self.create_subscription(
            Float32MultiArray,
            '/micro_control_target',
            self.get_target,
            1)
        self.micro_target_subscriber
        self.pid_send_current = self.create_publisher(Float32MultiArray,'/joint_cur',1)
    def get_target(self, msg):
        self.micro_target = np.array(msg.data)
        print(f"Moving to: {self.micro_target}")
        self.old_time = time.perf_counter()
        self.error_integration = 0.0
        if not self.start_moving:
            # Create a subscriber and publisher to control the robot
            self.control_subscriber = self.create_subscription(
                Float32MultiArray,
                '/joint_state',
                self.control_function,
                1)
            self.control_subscriber  # Dummy expression to avoid unused variable warning
            self.start_moving = True
    def control_function(self, msg):
        self.current_angle = np.array(msg.data)[0]
        self.current_error = self.micro_target - self.current_angle
        self.current_time = time.perf_counter()
        self.dt = self.current_time - self.old_time
        self.old_time = self.current_time
        self.de = (self.current_error - self.old_error)/self.dt
        self.old_error = self.current_error 
        self.error_integration += self.current_error*self.dt
        electrical_current = self.kp*self.current_error + self.ki*self.error_integration +  self.kd*self.de
        self.current_msg.data = electrical_current.tolist()
        self.pid_send_current.publish(self.current_msg)
        print(self.current_angle)

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    pid_angle_control = PIDAngleControl()  # Create an instance of the SimplePublisher
    print("Node started")
    try:
        rclpy.spin(pid_angle_control)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass
    pid_angle_control.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

if __name__ == '__main__':
    main()  # Execute the main function
