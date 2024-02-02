import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
from amr.module_my_math import Kinematics, Solver

class weighted_IK_Controller(Node):
    def __init__(self):
        super().__init__('angle_controller_with_jacobian_based_solver')
        # Custom parameters
        self.d_error_clamping = 10
        self.joint_weight = np.array([1,1,1])
        self.k_damped_factor = 1        
        # Declare needed variable
        # self.arm_kinematics = Kinematics([0,0,0]) #can comment out
        self.target = np.array([0, 0, 0])
        self.inverse_kinematic_solver = Solver()
        self.current_angles = None
        # self.start_moving = False
        # Create a subscriber to get the current state
        self.control_subscriber = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.update_current_state,
            10)
        self.control_subscriber  # Dummy expression to avoid unused variable warning

        # Create a subscriber to get user target
        self.command_target_subscriber = self.create_subscription(
            Float32MultiArray,
            '/control_target',
            self.control_function,
            1)
        self.command_target_subscriber
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos',10)

    # Update the current state
    def update_current_state(self, msg):
        # self.arm_kinematics.Update_Data(-np.array(msg.data)) #can comment out
        # print(self.target - self.arm_kinematics.arm_3_position) #can comment out
        self.current_angles = -np.array(msg.data)[0:3]
    # Control the robot
    def control_function(self, msg):
        # Get the target
        self.target = np.array(msg.data)
        print(f"Moving to: {self.target}")
        # Moving to the target
        target_angles = self.inverse_kinematic_solver.Solving_Inverse_Kinematics_using_Jacobian_arm_3(end_effector_target = self.target,
                                                                                                      initial_joint_angle = self.current_angles,
                                                                                                      weight_vector = self.joint_weight)
        target_angles_msg = Float32MultiArray()
        target_angles_msg.data = (-target_angles).tolist()
        self.subscriber_publisher.publish(target_angles_msg)
 
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    weighted_IK_Controller_node = weighted_IK_Controller()  # Create an instance of the SimpleSubscriber
    print("Node started")
    try:
        rclpy.spin(weighted_IK_Controller_node)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass
    weighted_IK_Controller_node.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library
# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function


