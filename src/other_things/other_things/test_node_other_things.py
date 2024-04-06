# # Importing necessary libraries from ROS2 Python client library
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
# from std_msgs.msg import Float32MultiArray
# import numpy as np

# # Defining the SimplePublisher class which inherits from Node
# class DataCollection(Node):

#     def __init__(self):
#         super().__init__('data_collection')  # Initialize the node with the name 'simple_publisher'
#         # Create a subscriber to get user target
#         self.command_target_subscriber = self.create_subscription(
#             Float32MultiArray,
#             '/joint_state',
#             self.get_command_target,
#             1)
#         self.command_target_subscriber
#         self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/micro_control_target', 1)
#     def get_command_target(self, msg):
#         print(np.array(msg.data)[0])
#         msg = Float32MultiArray()  # Creating a String message object
#         msg.data = [5.0, 5.0, 5.0]  # Setting the message data

#         self.subscriber_publisher.publish(msg)  # Publishing the message
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     simple_publisher = DataCollection()  # Create an instance of the SimplePublisher
#     # simple_publisher.publish()
#     print("Node started")
#     try:
#         rclpy.spin(simple_publisher)  # Keep the node alive and listening for messages
#     except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
#         pass

#     simple_publisher.destroy_node()  # Properly destroy the node
#     rclpy.shutdown()  # Shutdown the ROS2 Python client library

# # This condition checks if the script is executed directly (not imported)
# if __name__ == '__main__':
#     main()  # Execute the main function

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import numpy as np

# class UI_send_target(Node):
#     def __init__(self):
#         super().__init__('ui_send_target')
#         # Create a publisher
#         self.sendtarget = self.create_publisher(Float32MultiArray,'/micro_control_target',1)
        
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     UI_send_target_node = UI_send_target()  # Create an instance of the SimplePublisher
#     print("Node started")
#     try:
#         while True:
#             user_input = []
#             for i in range(3):
#                 user_input.append(float(input("Enter target: ")))
#             # user_input.append(0.0)
#             msg = Float32MultiArray()  # Creating a String message object
#             msg.data = user_input # Setting the message data
#             UI_send_target_node.sendtarget.publish(msg)
#             print(f"Target sent: {user_input}")
#     except KeyboardInterrupt:
#         pass
#     finally:
#         print()
#         print()
#         print("Thank you")
#         print()
#         print()
#         UI_send_target_node.destroy_node()  # Properly destroy the node
#         rclpy.shutdown()  # Shutdown the ROS2 Python client library
# if __name__ == '__main__':
#     main()  # Execute the main function




# ########################################## Data collection
# # Importing necessary libraries from ROS2 Python client library
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
# from std_msgs.msg import Float32MultiArray
# import time

# # Defining the SimplePublisher class which inherits from Node
# class DataCollection(Node):

#     def __init__(self):
#         super().__init__('data_collection')  # Initialize the node with the name 'simple_publisher'
#         # Create a subscriber to get user target
#         # self.command_target_subscriber = self.create_subscription(
#         #     Float32MultiArray,
#         #     '/control_target',
#         #     self.get_command_target,
#         #     1)
#         # self.command_target_subscriber
#         # self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos_rel',10)

#         self.publisher_ = self.create_publisher(Float32MultiArray, '/joint_vel', 1)
#         self.publisher_reset = self.create_publisher(Float32MultiArray, '/joint_pos', 1)
#         timer_period = 1  # Setting the timer period to 1 second
#         # Create a timer that calls the timer_callback method every 1 second
#         self.timer = self.create_timer(timer_period, self.publish)
#         # self.publish()
#     def publish(self):
#         # print(2)
#         # msg = Float32MultiArray()  # Creating a String message object
#         # msg.data = [0.0, 0.0, 0.0]  # Setting the message data
#         # self.publisher_reset.publish(msg)
#         msg = Float32MultiArray()  # Creating a String message object
#         sign = 1
#         msg.data = [90.0, -90.0, 90.0]  # Setting the message data
#         self.publisher_reset.publish(msg)  # Publishing the message
#         for i in range(0, 6):            
#             j = i*sign
#             # sign = -1*sign
#             msg.data = [-float(j), 0.0, 0.0]  # Setting the message data
#             self.publisher_.publish(msg)  # Publishing the message
#             # Logging the published message to the console
#             self.get_logger().info('Publishing: "%s"' % msg.data)
#             time.sleep(2)
#             msg.data = [float(j), 0.0, 0.0]  # Setting the message data
#             self.publisher_.publish(msg)  # Publishing the message
#             # Logging the published message to the console
#             self.get_logger().info('Publishing: "%s"' % msg.data)
#             time.sleep(2)
#         msg.data = [90.0, -90.0, 90.0]  # Setting the message data
#         self.publisher_reset.publish(msg)  # Publishing the message

# # The main function which serves as the entry point for the program
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     simple_publisher = DataCollection()  # Create an instance of the SimplePublisher
#     # simple_publisher.publish()
#     try:
#         rclpy.spin_once(simple_publisher)  # Keep the node alive and listening for messages
#     except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
#         pass

#     simple_publisher.destroy_node()  # Properly destroy the node
#     rclpy.shutdown()  # Shutdown the ROS2 Python client library

# # This condition checks if the script is executed directly (not imported)
# if __name__ == '__main__':
#     main()  # Execute the main function


# ########################################## Data collection
# # Importing necessary libraries from ROS2 Python client library
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
# from std_msgs.msg import Float32MultiArray
# import time

# # Defining the SimplePublisher class which inherits from Node
# class DataCollection(Node):

#     def __init__(self):
#         super().__init__('data_collection')  # Initialize the node with the name 'simple_publisher'
#         # Create a subscriber to get user target
#         # self.command_target_subscriber = self.create_subscription(
#         #     Float32MultiArray,
#         #     '/control_target',
#         #     self.get_command_target,
#         #     1)
#         # self.command_target_subscriber
#         # self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_pos_rel',10)

#         self.publisher_ = self.create_publisher(Float32MultiArray, '/joint_vel', 1)
#         self.publisher_reset = self.create_publisher(Float32MultiArray, '/joint_cur', 1)
#         timer_period = 1  # Setting the timer period to 1 second
#         # Create a timer that calls the timer_callback method every 1 second
#         self.timer = self.create_timer(timer_period, self.publish)
#         # self.publish()
#     def publish(self):
#         # print(2)
#         # msg = Float32MultiArray()  # Creating a String message object
#         # msg.data = [0.0, 0.0, 0.0]  # Setting the message data
#         # self.publisher_reset.publish(msg)
#         msg = Float32MultiArray()  # Creating a String message object
#         sign = 1
#         msg.data = [-10.8496, -11.3728, -15.6475]  # Setting the message data
#         self.publisher_reset.publish(msg)  # Publishing the message

# # The main function which serves as the entry point for the program
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     simple_publisher = DataCollection()  # Create an instance of the SimplePublisher
#     # simple_publisher.publish()
#     try:
#         rclpy.spin_once(simple_publisher)  # Keep the node alive and listening for messages
#     except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
#         pass

#     simple_publisher.destroy_node()  # Properly destroy the node
#     rclpy.shutdown()  # Shutdown the ROS2 Python client library

# # This condition checks if the script is executed directly (not imported)
# if __name__ == '__main__':
#     main()  # Execute the main function





# ##########################
# # Importing necessary libraries from ROS2 Python client library
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray  # Import the String message type from standard ROS2 message library
# import matplotlib.pyplot as plt
# import tkinter as tk
# import subprocess
# import time
# import numpy as np
# import matplotlib.pyplot as plt
# import subprocess
# from haptic.module_my_math import Kinematics
# import matplotlib.colors as mcolors
# import pickle


# # Defining the SimpleSubscriber class which inherits from Node
# class TestNode(Node):
#     def __init__(self):
#         super().__init__('simple_subscriber')  # Initialize the node with the name 'simple_subscriber'
#         # Create a subscription object that listens to messages of type String
#         # on the topic 'topic'. The 'listener_callback' function is called
#         # when a new message is received. '10' is the queue size.
#         self.get_current_position = self.create_subscription(
#             Float32MultiArray,
#             '/joint_state',
#             self.save_PNG,
#             1)
#         self.get_current_position  # Dummy expression to avoid unused variable warning

#         # Plotting variables
#         self.environment_name = "/home/pi/ros2_ws/src/haptic/haptic/Environment.pk1"
#         # Load the environment
#         with open(self.environment_name, 'rb') as file:
#             self.environment = pickle.load(file)
#         self.image_array = np.array(self.environment)[:,:,0]

#         self.PNG_export_name = "/home/pi/ros2_ws/src/haptic/haptic/PNG.jpeg"
#         custom_colors = ['#FF0000', '#00FF00', '#0000FF', '#FFFF00', '#FF00FF', '#00FFFF', '#FFA500', '#800080', '#008080', '#800000']
#         self.custom_cmap = mcolors.ListedColormap(custom_colors)

#         self.fig, self.ax = plt.subplots()
#         self.ax.set_xlim(0, 300)  # Replace xmin and xmax with your desired limits
#         self.ax.set_ylim(0, 600)  # Replace ymin and ymax with your desired limits
#         self.ax.set_aspect('equal')
#         self.ax.invert_yaxis()

#         self.arm_1_plot, = self.ax.plot("r-") 
#         self.arm_2_plot, = self.ax.plot("g-") 
#         self.arm_3_plot, = self.ax.plot("b-")
#         self.ax.imshow(self.image_array.astype(np.uint8), cmap=self.custom_cmap)
#         # self.ax.axis('off')
#         self.fig.savefig(self.PNG_export_name, bbox_inches='tight', pad_inches=0)
#         self.feh_process = subprocess.Popen(['feh', self.PNG_export_name])
#         self.feh_process2 = subprocess.Popen(['feh', self.PNG_export_name])
#         self.feh_process3 = subprocess.Popen(['feh', self.PNG_export_name])
#         # Declare robot handler
#         self.robot_state = Kinematics([0,0,0])
#         # Declare control alternating variable (mod3)
#         self.loop_state = 0


#     def save_PNG(self, msg):
#         start_time = time.perf_counter()
#         self.robot_state.Update_Data(-np.array(msg.data)[0:3])
#         self.arm_1_plot.set_data([0, self.robot_state.arm_1_position[0]], [300, 300-self.robot_state.arm_1_position[1]])
#         self.arm_2_plot.set_data([self.robot_state.arm_1_position[0], self.robot_state.arm_2_position[0]], [300-self.robot_state.arm_1_position[1], 300-self.robot_state.arm_2_position[1]])
#         self.arm_3_plot.set_data([self.robot_state.arm_2_position[0], self.robot_state.arm_3_position[0]], [300-self.robot_state.arm_2_position[1], 300-self.robot_state.arm_3_position[1]])
#         self.fig.savefig(self.PNG_export_name)
#         # self.feh_process2.terminate()
#         # self.feh_process2 = subprocess.Popen(['feh', self.PNG_export_name])
#         if self.loop_state == 0:
#             self.feh_process.terminate()
#             self.feh_process = subprocess.Popen(['feh', self.PNG_export_name])
#             self.loop_state = (self.loop_state+1)%3
#         elif self.loop_state == 1:
#             self.feh_process2.terminate()
#             self.feh_process2 = subprocess.Popen(['feh', self.PNG_export_name])
#             self.loop_state = (self.loop_state+1)%3
#         elif self.loop_state == 2:
#             self.feh_process3.terminate()
#             self.feh_process3 = subprocess.Popen(['feh', self.PNG_export_name])
#             self.loop_state = (self.loop_state+1)%3
#         # print(self.feh_process2)
#         # print(self.feh_process)
#         # self.feh_process2.terminate()        
#         # end_time = time.perf_counter()
#         end_time = time.perf_counter()
#         print(f"Time need: {end_time - start_time}")

# # The main function which serves as the entry point for the program
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     simple_subscriber = TestNode()  # Create an instance of the SimpleSubscriber
#     print("Node started")
#     try:
#         rclpy.spin(simple_subscriber)  # Keep the node alive and listening for messages
#     except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
#         pass

#     simple_subscriber.destroy_node()  # Properly destroy the node
#     rclpy.shutdown()  # Shutdown the ROS2 Python client library

# # This condition checks if the script is executed directly (not imported)
# if __name__ == '__main__':
#     main()  # Execute the main function











# ##########################
# # Importing necessary libraries from ROS2 Python client library
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray  # Import the String message type from standard ROS2 message library
# import matplotlib.pyplot as plt
# import tkinter as tk
# import subprocess
# import time
# import numpy as np
# import matplotlib.pyplot as plt
# import subprocess
# from haptic.module_my_math import Kinematics, Solver
# import matplotlib.colors as mcolors
# import pickle


# # Defining the SimpleSubscriber class which inherits from Node
# class TestNode(Node):
#     def __init__(self):
#         super().__init__('simple_subscriber')  # Initialize the node with the name 'simple_subscriber'
#         # Create a subscription object that listens to messages of type String
#         # on the topic 'topic'. The 'listener_callback' function is called
#         # when a new message is received. '10' is the queue size.
#         self.get_current_position = self.create_subscription(
#             Float32MultiArray,
#             '/joint_state',
#             self.control_position,
#             1)
#         self.get_current_position  # Dummy expression to avoid unused variable warning
#         self.subscriber_publisher = self.create_publisher(Float32MultiArray,'/joint_vel', 1)
#         self.publish_msg = Float32MultiArray()  # Creating a String message object

#         self.robot_state = Kinematics([0,0,0])
#         self.optimizing_joint_state_solver = Solver()
#         self.optimizing_joint_state_solver.Set_Variable_for_Maximizing_Distance_to_Joint_Limit_Solver()

#     def control_position(self, msg):
#         start_time = time.perf_counter()
#         joint_state = -np.array(msg.data)[0:3]
#         self.robot_state.Update_Data(joint_state)

#         # controlled_joint_state = self.optimizing_joint_state_solver.Solving_Inverse_Kinematics_while_Maximizing_Distance_to_Joint_limit(, joint_state)

#         controlled_joint_state = self.optimizing_joint_state_solver.Solving_Inverse_Kinematics_while_Maximizing_Distance_to_Joint_limit(np.array([200,0,0]), joint_state)     
#         # controlled_joint_state = self.optimizing_joint_state_solver.Solving_Inverse_Kinematics_while_Maximizing_Distance_to_Joint_limit(np.array(self.robot_state.arm_3_position), joint_state)     

#         controlled_joint_state = controlled_joint_state/np.pi*180 - joint_state
 


#         controlled_joint_state = -controlled_joint_state/10
#         self.publish_msg.data = controlled_joint_state.tolist()  # Setting the message data
#         self.subscriber_publisher.publish(self.publish_msg)
#         end_time = time.perf_counter()
#         print(f"Time need: {end_time - start_time}")

# # The main function which serves as the entry point for the program
# def main(args=None):
#     rclpy.init(args=args)  # Initialize the ROS2 Python client library
#     simple_subscriber = TestNode()  # Create an instance of the SimpleSubscriber
#     print("Node started")
#     try:
#         rclpy.spin(simple_subscriber)  # Keep the node alive and listening for messages
#     except KeyboardInterrupt:  # Allow the program to exit on a keyboard interrupt (Ctrl+C)
#         pass

#     simple_subscriber.destroy_node()  # Properly destroy the node
#     rclpy.shutdown()  # Shutdown the ROS2 Python client library

# # This condition checks if the script is executed directly (not imported)
# if __name__ == '__main__':
#     main()  # Execute the main function


# import rclpy
# import rclpy.node

# class MinimalParam(rclpy.node.Node):
#     def __init__(self):
#         super().__init__('minimal_param_node')

#         self.declare_parameter('my_parameter', 'world')

#         self.timer = self.create_timer(1, self.timer_callback)

#     def timer_callback(self):
#         my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

#         self.get_logger().info('Hello %s!' % my_param)

#         # my_new_param = rclpy.parameter.Parameter(
#         #     'my_parameter',
#         #     rclpy.Parameter.Type.STRING,
#         #     'world'
#         # )
#         # all_new_parameters = [my_new_param]
#         # self.set_parameters(all_new_parameters)

# def main():
#     rclpy.init()
#     node = MinimalParam()
#     rclpy.spin(node)

# if __name__ == '__main__':
#     main()






























import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String  # Import the String message type from standard ROS2 message library
from std_msgs.msg import Float32MultiArray
from haptic.module_my_math import Kinematics

class weighted_IK_Controller(Node):
    def __init__(self):
        super().__init__('controller_weighted_inverse_kinematic')
        # Set control parameters
        self.total_damping = 3
        self.d_error_clamping = 10*self.total_damping
        self.joint_weight = np.array([1,1,1])
        self.k_damped_factor = 1        
        # Declare PID variable
        self.kp = 1
        self.ki = 0.3
        self.kd = 0.3
        self.task_space_distance_limit = self.d_error_clamping
        self.old_error = 0
        self.old_time = time.perf_counter()

        self.error_integration = 0.0
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
        if np.sum(target_velocity**2) < 3:
            target_velocity = np.array([0.0,0.0,0.0])
        # target_velocity = (np.abs(target_velocity)>0.3)*target_velocity
        self.target_velocity_msg.data = target_velocity.tolist()
        self.subscriber_publisher.publish(self.target_velocity_msg)
        # print(self.arm_kinematics.arm_3_position)
        # print(target_velocity)
        # print(self.target - self.arm_kinematics.arm_3_position)
        self.logger.info(f"Speed {target_velocity}")        
        self.logger.info(f"dx {self.current_error}")        
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


