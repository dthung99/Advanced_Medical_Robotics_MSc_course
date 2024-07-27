############################## NEWWWWWWWWWWW

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
from haptic.module_my_math import Kinematics
import numpy as np

class weighted_IK_Controller(Node):
    def __init__(self):
        super().__init__('controller_weighted_inverse_kinematic')
        # Set control parameters
        self.total_damping = 3
        self.d_error_clamping = 30*self.total_damping
        self.joint_weight = np.array([1,1,1])
        self.k_damped_factor = 1        
        # Declare PID variable
        self.kp = 1
        self.ki = 0.0
        self.kd = 0.0
        self.ki = 0.3
        self.kd = 0.3
        self.task_space_distance_limit = self.d_error_clamping
        self.old_error = 0
        self.old_time = time.perf_counter()

        self.error_integration = 0.04
        self.integral_clamping_condition = 1

        # Declare needed variables
        self.arm_kinematics = Kinematics([0,0,0])
        self.start_moving = False

        self.logger = self.get_logger()
        self.target_velocity_msg = Float32MultiArray()

        # Create a subscriber to get user target
        self.command_target_subscriber = self.create_subscription(
            Float32MultiArray,
            '/control_target',
            self.get_command_target,
            1)
        self.command_target_subscriber
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/micro_control_target',1)
    # Getting the target
    def get_command_target(self, msg):
        self.target = np.array(msg.data)
        self.logger.info(f"Moving to: {self.target}")
        self.error_integration = 0.0
        self.integral_clamping_condition = 1
        if not self.start_moving:
            # Create a subscriber and publisher to control the robot
            self.control_subscriber = self.create_subscription(
                Float32MultiArray,
                '/joint_state',
                self.control_function,
                1)
            self.control_subscriber  # Dummy expression to avoid unused variable warning
            self.start_moving = True
    # Control the robot
    def control_function(self, msg):
        self.arm_kinematics.Update_Data(-np.array(msg.data))
        self.current_error = self.target - self.arm_kinematics.arm_3_position
        error_length = np.linalg.norm(self.current_error)
        if error_length < 5:
            self.logger.info(f"Error: {round(error_length,1)}mm")        
            self.target_velocity_msg.data = [0.0, 0.0, 0.0]
            self.subscriber_publisher.publish(self.target_velocity_msg)
            return
        self.current_time = time.perf_counter()
        self.dt = self.current_time - self.old_time
        self.old_time = self.current_time
        self.de = (self.current_error - self.old_error)/self.dt
        self.old_error = self.current_error 
        self.error_integration += self.current_error*self.dt*self.integral_clamping_condition
        PID_task_space_diff_output = self.kp*self.current_error + self.ki*self.error_integration + self.kd*self.de
        logic_for_reset = np.abs(PID_task_space_diff_output) > self.task_space_distance_limit
        PID_task_space_diff_output = PID_task_space_diff_output - (PID_task_space_diff_output - self.task_space_distance_limit*np.sign(PID_task_space_diff_output))*logic_for_reset
        self.integral_clamping_condition = 1-(np.sign(PID_task_space_diff_output)==np.sign(self.current_error))*logic_for_reset
        target_velocity = self.arm_kinematics.get_Weighted_Velocity_needed_for_Target_From_Task_Space_Difference(PID_task_space_diff_output,
                                                                                                                 d_error_clamping = self.d_error_clamping,
                                                                                                                 weight_vector = self.joint_weight,
                                                                                                                 k_damped_factor = self.k_damped_factor)/self.total_damping
        target_velocity = -target_velocity*180/np.pi
        # target_velocity = (np.abs(target_velocity)>0.3)*target_velocity
        self.target_velocity_msg.data = target_velocity.tolist()
        self.subscriber_publisher.publish(self.target_velocity_msg)
        # print(self.arm_kinematics.arm_3_position)
        # print(target_velocity)
        # print(self.target - self.arm_kinematics.arm_3_position)
        # self.logger.info(f"Speed {target_velocity}")        
        self.logger.info(f"Error: {round(error_length,1)}mm")        
        # self.logger.info(f"target {self.target}")        
 
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


