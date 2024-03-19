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
        # self.friction_coef = np.array([[0.7336, 1.3592, 0.6990],
        #                               [23.0210, 13.7851, 23.4776]])
        self.friction_coef = np.array([[1.8632, 1.2933, 1.1437],
                                       [10.8496, 11.3728, 15.6475]])
        # self.friction_coef = np.array([1.0666, 10.2892])
        self.dynamic_weird_scale = 1 #200
        self.stable_factor = 1
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
        self.previous_time = time.perf_counter()
        self.previous_joint_velocity = np.array([0, 0, 0])
        # # Dynamic model
        self.dynamic_model = Dynamics_Model()
        # self.dynamic_model.update_Physical_Properties(armlength = np.array([0.1, 0.1, 0.1]),
        #                                               armlength_to_center = np.array([0.071, 0.071, 0.052]),
        #                                               moment_of_inertia = np.array([0.0000535, 0.0000535, 0.00002487]),
        #                                               mass_of_arm = np.array([0.0454, 0.0454, 0.02738]))
        self.dynamic_model.update_Physical_Properties(armlength = np.array([0.1, 0.1, 0.1]),
                                                      armlength_to_center = np.array([0.071, 0.071, 0.052]),
                                                      moment_of_inertia = np.array([0.0000535, 0.0000535, 0.00002487]),
                                                      mass_of_arm = np.array([0.0454, 0.0454, 0.02738]))
        self.force_publisher = self.create_publisher(Float32MultiArray,'/joint_cur',1)
        self.force_msg = Float32MultiArray()

    def update_force(self, msg):
        start_time = time.perf_counter()
        current_time = time.perf_counter()
        three_joint_states = -np.array(msg.data)
        joint_angle = three_joint_states[0:3]
        joint_velocity = three_joint_states[3:6]

        joint_accerleration = (joint_velocity - self.previous_joint_velocity)/(current_time - self.previous_time)
        self.previous_time = current_time
        self.previous_joint_velocity = joint_velocity       

        end_effector_velocity = np.matmul(self.robot_state.Jacobian(), joint_velocity.reshape(-1,)/180*np.pi)/1000

        # Get state
        self.robot_state.Update_Data(joint_angle)
        self.dynamic_model.update_Model_9_Kinematics(kinematic_vector=np.r_[joint_angle, joint_velocity, joint_accerleration]/180*np.pi)

        # Get environment information
        x_position = round(self.robot_state.arm_3_position[0])
        y_position = round(self.robot_state.arm_3_position[1])
        if ((300-y_position)<600) and (300-y_position)>0 and x_position>0 and x_position<300:
            environmentvector = np.array(self.environment[300 - y_position][x_position])
        else:
            environmentvector = np.zeros((10,))
        # Calculate force
        force_calculated = np.array([0, 0, 0])
        if np.sum(environmentvector*environmentvector) == 0:
            # Compensate for friction
            force_calculated = force_calculated + joint_velocity*self.friction_coef[0] + np.sign(joint_velocity)*self.friction_coef[1]
            # Compensate for dynamic model
            # force_calculated = force_calculated + 
            force_calculated = force_calculated + self.dynamic_model.get_Torque_from_Motor_Kinematics()/self.motor_scale*1000*self.dynamic_weird_scale
        else:
            # Get pure force
            brute_force = (environmentvector[1:3] + end_effector_velocity*environmentvector[3]/self.motor_scale)/self.stable_factor
            # Get environment force
            force_calculated = force_calculated + self.robot_state.get_Torque_from_End_effector_Wrench(brute_force/self.motor_scale).reshape(-1,)
            # Get damping factor
            # Compensate for friction
            # force_calculated = force_calculated + joint_velocity*self.friction_coef[0] + np.sign(joint_velocity)*self.friction_coef[1]
            # Compensate for dynamic model
            # force_calculated = force_calculated + 
            # force_calculated = force_calculated + self.dynamic_model.get_Torque_from_Motor_Kinematics()/self.motor_scale*1000*self.dynamic_weird_scale
            # No B matrix
            # force_calculated = force_calculated + self.dynamic_model.get_Torque_from_Motor_Kinematics_No_inertia_Matrix()/self.motor_scale*1000*self.dynamic_weird_scale

        print(force_calculated)
        print(self.dynamic_model.get_Torque_from_Motor_Kinematics()/self.motor_scale*1000*self.dynamic_weird_scale)

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

        end_time = time.perf_counter()
        print("Execution time:", end_time - start_time, "seconds")


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
