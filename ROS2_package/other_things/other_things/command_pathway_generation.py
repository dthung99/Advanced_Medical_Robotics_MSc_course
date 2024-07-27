import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from haptic.module_my_math import Kinematics

class linear_Pathway_Generation(Node):
    def __init__(self):
        super().__init__('pathway')
        # Custom parameters
        self.linear_step_length = 30
        self.via_point_acceptable_error = 30
        # Declare kinematics
        self.arms_kinematic = Kinematics([0, 0, 0])
        # Subscriber to get current state and update pathway
        self.get_current_position_subscriber = self.create_subscription(
            Float32MultiArray,
            '/joint_state',
            self.get_Current_Position,
            10)
        self.get_current_position_subscriber  # Dummy expression to avoid unused variable warning
        # Subscriber to get user target input
        self.get_target_subscriber = self.create_subscription(
            Float32MultiArray,
            '/final_target',
            self.set_Target_and_Generate_Pathway,
            10)
        self.get_current_position_subscriber  # Dummy expression to avoid unused variable warning
        self.pathway = []
        self.update_pathway = self.create_publisher(Float32MultiArray, '/control_target', 10)

    # Get current arm position that run in every loop
    def get_Current_Position(self, msg):
        self.arms_kinematic.Update_Data(-np.array(msg.data))
        if len(self.pathway) > 1:
            if np.linalg.norm(self.arms_kinematic.arm_3_position - self.pathway[0]) < self.via_point_acceptable_error:
                via_point_msg = Float32MultiArray()
                self.pathway.pop(0)
                via_point_msg.data = self.pathway[0].tolist()
                self.update_pathway.publish(via_point_msg)

    # Run after receiving the target and generate a pathway
    def set_Target_and_Generate_Pathway(self, msg):
        origin = self.arms_kinematic.arm_3_position
        # Generate list of point on pathway
        target = np.array(msg.data)
        unit_step = target - origin
        pathway_length = np.linalg.norm(unit_step)
        max_number_of_step = pathway_length/self.linear_step_length
        unit_step = unit_step/pathway_length
        self.pathway = []
        i = 0
        while i < max_number_of_step:
            self.pathway.append(origin + i*unit_step*self.linear_step_length)
            i += 1
        self.pathway.append(target)

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    linear_Pathway_Generation_node = linear_Pathway_Generation()  # Create an instance of the SimplePublisher
    print("Node started")
    try:
        rclpy.spin(linear_Pathway_Generation_node)
    except KeyboardInterrupt:
        pass
    finally:
        linear_Pathway_Generation_node.destroy_node()
    rclpy.shutdown()  # Shutdown the ROS2 Python client library
if __name__ == '__main__':
    main()  # Execute the main function
