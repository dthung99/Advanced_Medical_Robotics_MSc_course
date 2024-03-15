# Importing necessary libraries from ROS2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray

# Defining the SimplePublisher class which inherits from Node
class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')  # Initialize the node with the name 'simple_publisher'
        # Create a publisher object with String message type on the topic 'advanced_topic'
        # The second argument '10' is the queue size
        self.publisher_ = self.create_publisher(Float32MultiArray, '/joint_state', 10)
        timer_period = 1  # Setting the timer period to 1 second
        # Create a timer that calls the timer_callback method every 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        print("请输入3个电机的目标角度（用空格分隔）：")
        input_angles = input().split()
        
        # 检查输入的角度数量是否正确
        if len(input_angles) != 3:
            print("请输入正确数量的角度！")
            return
        
        # 将输入的角度转换为浮点数
        try:
            motor_angles = [float(angle) for angle in input_angles]
        except ValueError:
            print("请输入有效的数字！")
            return

        # 创建 Float32MultiArray 消息
        msg = Float32MultiArray()
        msg.data = motor_angles

        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing motor angles: {}'.format(msg.data))
        
# The main function which serves as the entry point for the program
def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    simple_publisher = SimplePublisher()  # Create an instance of the SimplePublisher

    try:
        rclpy.spin(simple_publisher)  # Keep the node alive and listening for messages
    except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
        pass

    simple_publisher.destroy_node()  # Properly destroy the node
    rclpy.shutdown()  # Shutdown the ROS2 Python client library

# This condition checks if the script is executed directly (not imported)
if __name__ == '__main__':
    main()  # Execute the main function