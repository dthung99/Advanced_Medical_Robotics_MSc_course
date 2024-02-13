import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class PIDAngleControl_Combine_Publisher(Node):
    def __init__(self):
        super().__init__('pidangleControl_combine_publisher')
        # Declare variables
        self.target_1 = 0.0
        self.target_2 = 0.0
        self.target_3 = 0.0
        # Subscriber to get target current
        self.get_target_current_1 = self.create_subscription(
            Float32MultiArray,
            '/micro_control_target_1',
            self.update_Target_1,
            1)
        self.get_target_current_1
        self.get_target_current_2 = self.create_subscription(
            Float32MultiArray,
            '/micro_control_target_2',
            self.update_Target_2,
            1)
        self.get_target_current_2
        self.get_target_current_3 = self.create_subscription(
            Float32MultiArray,
            '/micro_control_target_3',
            self.update_Target_3,
            1)
        self.get_target_current_3
        self.get_current_state = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.send_desired_currents,
            1)
        self.get_current_state
        self.pid_send_combined_current = self.create_publisher(Float32MultiArray,'/joint_cur',1)
        # Create global msg
        self.current_msg = Float32MultiArray()  # Creating a String message object

    def update_Target_1(self, msg):
        self.target_1 = msg.data
    def update_Target_2(self, msg):
        self.target_2 = msg.data
    def update_Target_3(self, msg):
        self.target_3 = msg.data
    def send_desired_currents(self, msg):
        # self.current_msg.data = 
        data = np.array([self.target_1, self.target_2, self.target_3]).reshape(-1,)
        data = data.tolist()
        self.current_msg.data = data
        self.pid_send_combined_current.publish(self.current_msg)
        print(self.current_msg.data)

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    combined_PIDangle_control = PIDAngleControl_Combine_Publisher()  # Create an instance of the SimplePublisher
    print("Node started")
    try:
        rclpy.spin(combined_PIDangle_control)
    except KeyboardInterrupt:
        pass
    finally:
        combined_PIDangle_control.destroy_node()
    rclpy.shutdown()  # Shutdown the ROS2 Python client library
if __name__ == '__main__':
    main()  # Execute the main function
