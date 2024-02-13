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
            10)
        self.get_target_current_1
        self.get_target_current_2 = self.create_subscription(
            Float32MultiArray,
            '/micro_control_target_2',
            self.update_Target_2,
            10)
        self.get_target_current_2
        self.get_target_current_3 = self.create_subscription(
            Float32MultiArray,
            '/micro_control_target_3',
            self.update_Target_3,
            10)
        self.get_target_current_3

    def update_Target_1(self, msg):
        self.target_1 = msg.data
    def update_Target_2(self, msg):
        self.target_2 = msg.data
    def update_Target_3(self, msg):
        self.target_3 = msg.data





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
