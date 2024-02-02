# Importing necessary libraries from ROS2 Python client library
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
from amr.module_kinematics import Forward_Kinematics

# Defining the SimpleSubscriber class which inherits from Node
class IkController(Node):

    def __init__(self):
        super().__init__('ik_controller')  # Initialize the node with the name 'simple_subscriber'
        # Create a subscription object that listens to messages of type String
        # on the topic 'topic'. The 'listener_callback' function is called
        # when a new message is received. '10' is the queue size.
        self.arm_kinematics = Forward_Kinematics([0,0,0])
        self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos_rel',10)


        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.listener_callback,
            10)
        self.subscription  # Dummy expression to avoid unused variable warning


    def listener_callback(self, msg):
        # print("Hi")
        # Callback function that is invoked when a new message is received
        arm1Angle = msg.data[0]
        arm2Angle = msg.data[1]
        arm3Angle = msg.data[2]
        self.arm_kinematics.Update_Data([-arm1Angle,-arm2Angle,-arm3Angle])
        J = self.arm_kinematics.Jacobian()
        IvJ = np.matmul(J.T,np.linalg.inv(np.matmul(J,J.T)))

        Target = np.array([299,0,0])
        delta = Target - self.arm_kinematics.arm_3_position
        delta = delta[0:2]
        # print(delta)
        h = np.matmul(IvJ,delta.reshape(-1,1)).flatten()
        h = -h*180/np.pi
        #self.get_logger().info('"%s"'% msg.data[0])  # Log the received message
        Position_msg = Float32MultiArray()
        Position_msg.data = h.tolist()
        # Position_msg.data = []
        # # print(Position_msg.data)
        # # print(h.tolist())
        self.subscriber_publisher.publish(Position_msg)

        # #print("hello")

    


# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    simple_subscriber = IkController()  # Create an instance of the SimpleSubscriber

    try:
        rclpy.spin(simple_subscriber)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    simple_subscriber.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function


