import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
from amr.module_my_math import Kinematics

class weighted_IK_Controller(Node):
    def __init__(self):
        super().__init__('controller_weighted_inverse_kinematic')
        self.arm_kinematics = Kinematics([0,0,0])
        self.start_moving = False
        # Set control parameter
        self.d_error_clamping = 10
        self.joint_weight = np.array([1,1,1])
        self.k_damped_factor = 1        
        # Create a subscriber to get user target
        self.command_target_subscriber = self.create_subscription(
            Float32MultiArray,
            '/control_target',
            self.get_command_target,
            1)
        self.command_target_subscriber
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos_rel',10)
    # Getting the target
    def get_command_target(self, msg):
        self.target = np.array(msg.data)
        print(f"Moving to: {self.target}")
        if not self.start_moving:
            # Create a subscriber and publisher to control the robot
            self.control_subscriber = self.create_subscription(
                Float32MultiArray,
                '/joint_state',
                self.control_function,
                10)
            self.control_subscriber  # Dummy expression to avoid unused variable warning
            self.start_moving = True
    # Control the robot
    def control_function(self, msg):
        self.arm_kinematics.Update_Data(-np.array(msg.data))
        target_velocity = self.arm_kinematics.get_Weighted_Velocity_needed_for_Target(self.target,
                                                                                      d_error_clamping = self.d_error_clamping,
                                                                                      weight_vector = self.joint_weight,
                                                                                      k_damped_factor = self.k_damped_factor)
        target_velocity = -target_velocity*180/np.pi
        target_velocity_msg = Float32MultiArray()
        target_velocity_msg.data = target_velocity.tolist()
        self.subscriber_publisher.publish(target_velocity_msg)
        # print(self.arm_kinematics.arm_3_position)
        # print(target_velocity)
        # print(self.target - self.arm_kinematics.arm_3_position)
 
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


