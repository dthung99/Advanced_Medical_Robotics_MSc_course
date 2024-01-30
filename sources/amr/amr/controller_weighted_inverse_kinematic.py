import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
from amr.module_kinematics import Forward_Kinematics

# Defining the SimpleSubscriber class which inherits from Node
class weighted_IK_Controller(Node):
    def __init__(self):
        super().__init__('controller_weighted_inverse_kinematic')
        self.arm_kinematics = Forward_Kinematics([0,0,0])
        self.start_moving = False
        # Create a subscriber to get user target
        self.command_target_subscriber = self.create_subscription(
            Float32MultiArray,
            '/ui_message',
            self.get_command_target,
            1)
        self.command_target_subscriber
        # Create a subscriber and publisher to control the robot
        self.control_subscriber = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.control_function,
            10)
        self.control_subscriber  # Dummy expression to avoid unused variable warning
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos_rel',10)
    # Getting the target
    def get_command_target(self, msg):
        self.target = np.array(msg.data)
        print(f"Moving to: {self.target}")
        self.start_moving = True

    # Control the robot
    def control_function(self, msg):
        if not self.start_moving:
            return
        self.arm_kinematics.Update_Data(-np.array(msg.data))
        jacobian = self.arm_kinematics.Jacobian()
        # Calculate the pseudo inverse: J^T*inv(J*JT)
        pseudo_inv = np.matmul(jacobian.T,np.linalg.inv(np.matmul(jacobian,jacobian.T)))
        task_space_vel = self.target - self.arm_kinematics.arm_3_position
        task_space_vel = task_space_vel[0:2]
        # Calculate the target velocity
        target_velocity = np.matmul(pseudo_inv,task_space_vel.reshape(-1,1)).flatten()
        # Scale to degree and send back to microcontroller
        target_velocity = -target_velocity*180/np.pi
        target_velocity_msg = Float32MultiArray()
        target_velocity_msg.data = target_velocity.tolist()
        self.subscriber_publisher.publish(target_velocity_msg)
 
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


