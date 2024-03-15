# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from haptic.module_my_math import Kinematics, Dynamics_Model
import numpy as np
import pickle
import time

# Defining the SimpleSubscriber class which inherits from Node
class Force_Control(Node):
    def __init__(self):
        super().__init__('force')
        # Declare parameter
        self.motor_scale = 1.487
        self.friction_coef = np.array([1.0666, 10.2892])
        self.environment_name = "/home/pi/ros2_ws/src/haptic/haptic/Environment.pk1"
        # Load the environment
        with open(self.environment_name, 'rb') as file:
            self.environment = pickle.load(file)
        # 
        self.get_current_position = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.update_force,
            1)
        self.get_current_position  # Dummy expression to avoid unused variable warning
        self.robot_state = Kinematics([0,0,0])
        # # Dynamic model
        self.dynamic_model = Dynamics_Model()
        self.dynamic_model.update_Physical_Properties(armlength = np.array([0.1, 0.1, 0.1]),
                                                      armlength_to_center = np.array([0.071, 0.071, 0.071]),
                                                      moment_of_inertia = np.array([0.0000535, 0.0000535, 0.0000535]),
                                                      mass_of_arm = np.array([0.0454, 0.0454, 0.0454]))
        self.force_publisher = self.create_publisher(Float32MultiArray,'/joint_cur',1)
        self.force_msg = Float32MultiArray()

    def update_force(self, msg):
        three_joint_states = -np.array(msg.data)
        joint_angle = three_joint_states[0:3]
        joint_velocity = three_joint_states[3:6]
        joint_accerleration = three_joint_states[6:9]
        # Get state
        self.robot_state.Update_Data(joint_angle)
        self.dynamic_model.update_Model_9_Kinematics(kinematic_vector=three_joint_states)

        # Get environment information
        x_position = round(self.robot_state.arm_3_position[0])
        y_position = round(self.robot_state.arm_3_position[1])
        brute_force = np.array(self.environment[300 - y_position][x_position][1:3])
        # Calculate force
        force_calculated = np.array([0, 0, 0])
        # Get environment force
        force_calculated = force_calculated + self.robot_state.get_Torque_from_End_effector_Wrench(brute_force/self.motor_scale).reshape(-1,)
        # Compensate for friction
        force_calculated = force_calculated + joint_velocity*self.friction_coef[0] + np.sign(joint_velocity)*self.friction_coef[1]
        # Compensate for dynamic model
        # force_calculated = force_calculated + 
        force_calculated = force_calculated + self.dynamic_model.get_Torque_from_Motor_Kinematics()/self.motor_scale
        # print(force_calculated)

        # print(force_calculated)

        # Print something
        # print(len(self.environment_name[x_position][y_position])  
        # print(300 - y_position)
        # print(brute_force)  #Frame convertion (150-y, x)

        # Send force
        force_calculated = -force_calculated
        self.force_msg.data = force_calculated.tolist()
        # self.force_msg.data = [500.0, 500.0, 500.0]
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
