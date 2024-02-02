# Importing necessary libraries from ROS2 Python client library
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
from amr.module_kinematics import Forward_Kinematics

# Defining the control_pathway_generation class which inherits from Node
class control_pathway_generation(Node):
    def __init__(self):
        super().__init__('control_pathway_generation')  # Initialize the node with the name 'control_pathway_generation'
        # Create a subscription object that listens to messages of type String
        # on the topic 'topic'. The 'listener_callback' function is called
        # when a new message is received. '10' is the queue size.
        self.number_points = 5
        self.arm_kinematics = Forward_Kinematics([0,0,0])
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos_rel',10)
        self.target = np.array([200,200,0])
        self.pathway = []
        self.start = True
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.listener_callback,
            10)
        self.subscription  # Dummy expression to avoid unused variable warning

    def listener_callback(self, msg):
        if self.start == True:
            self.arm_kinematics.Update_Data([-msg.data[0],-msg.data[1],-msg.data[2]])
            for i in range(self.number_points):
                self.pathway.append(self.arm_kinematics.arm_3_position + (self.target - self.arm_kinematics.arm_3_position)/self.number_points*(i+1))       
            print(self.pathway)
            self.current_target = self.pathway.pop(0)            
            self.start = False
        if np.linalg.norm(self.arm_kinematics.arm_3_position-self.current_target) < 10:
            if len(self.pathway)>1:
                self.current_target = self.pathway.pop(0)
        print(len(self.pathway))        
        print(self.arm_kinematics.arm_3_position)
        # Callback function that is invoked when a new message is received
        print(self.current_target)
        self.arm_kinematics.Update_Data([-msg.data[0],-msg.data[1],-msg.data[2]])
        jacobian = self.arm_kinematics.Jacobian()
        inverse_jacobian = np.matmul(jacobian.T,np.linalg.inv(np.matmul(jacobian,jacobian.T)))
        delta_xyz = self.current_target - self.arm_kinematics.arm_3_position
        delta_xyz = delta_xyz[0:2]
        target_speed = np.matmul(inverse_jacobian,delta_xyz.reshape(-1,1)).flatten()
        target_speed = -target_speed*180/np.pi
        Position_msg = Float32MultiArray()
        Position_msg.data = target_speed.tolist()
        self.subscriber_publisher.publish(Position_msg)


# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    control_pathway = control_pathway_generation()  # Create an instance of the control_pathway_generation
    try:
        rclpy.spin(control_pathway)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass
    control_pathway.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function


