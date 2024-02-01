import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys

class UI_send_target(Node):
    def __init__(self):
        super().__init__('ui_send_target')
        # Create a publisher
        self.sendtarget = self.create_publisher(Float32MultiArray,'/final_target',1)
        
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    UI_send_target_node = UI_send_target()  # Create an instance of the SimplePublisher
    try:
        while True:
            user_input = []
            for i in range(2):
                user_input.append(input("Enter target: "))
            user_input.append(0)
            msg = Float32MultiArray()  # Creating a String message object
            msg.data = [float(x) for x in user_input] # Setting the message data
            UI_send_target_node.sendtarget.publish(msg)
            print(f"Target sent: {user_input}")
    except KeyboardInterrupt:
        pass
    finally:
        print()
        print()
        print("Thank you")
        print()
        print()
        UI_send_target_node.destroy_node()  # Properly destroy the node
        rclpy.shutdown()  # Shutdown the ROS2 Python client library
if __name__ == '__main__':
    main()  # Execute the main function
