# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from haptic.module_my_math import Kinematics
import numpy as np

# Defining the SimpleSubscriber class which inherits from Node
class Force_Control(Node):
    def __init__(self):
        super().__init__('force')
        # Declare parameter
        self.motor_scale = 1.487
        self.friction_coef = np.array([1.0666, 10.2892])
        self.get_current_position = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.update_force,
            1)
        self.get_current_position  # Dummy expression to avoid unused variable warning
        self.robot_state = Kinematics([0,0,0])
        self.force_publisher = self.create_publisher(Float32MultiArray,'/joint_cur',1)
        self.force_msg = Float32MultiArray()

    def update_force(self, msg):
        # Get state
        self.robot_state.Update_Data(-np.array(msg.data[0:3]))
        current_velocity = -np.array(msg.data[3:6])
        # Calculate force1
        force_calculated = self.robot_state.get_Torque_from_End_effector_Wrench(np.array([0.0, 2.5])).reshape(-1,)/self.motor_scale
        # print(force_calculated)
        # force_calculated = force_calculated + current_velocity*self.friction_coef[0] + np.sign(current_velocity)*self.friction_coef[1]
        # Send force
        force_calculated = -force_calculated
        self.force_msg.data = force_calculated.tolist()
        self.force_publisher.publish(self.force_msg)

# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    Force_Control_node = Force_Control()  # Create an instance of the SimpleSubscriber
    print("Node started")
    try:
        rclpy.spin(Force_Control_node)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    Force_Control_node.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function
