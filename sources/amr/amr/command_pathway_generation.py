import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from amr.module_my_math import Kinematics

class linear_Pathway_Generation(Node):
    def __init__(self):
        super().__init__('pathway')
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

    def get_Current_Position(self, msg):
        self.arms_kinematic.Update_Data(-np.array(msg.data))
        while len(self.pathway) > 1:
            print(2)
            if np.linalg.norm(self.arms_kinematic.arm_3_position - self.pathway[0]) < 5:
                msg = Float32MultiArray
                msg.data = self.pathway.pop(0).tolist()
                self.update_pathway.publish(msg)

    def set_Target_and_Generate_Pathway(self, msg, step: float = 5):
        origin = self.arms_kinematic.arm_3_position
        # Generate list of point on pathway
        unit_step = np.array(msg) - origin
        print(unit_step)
        pathway_length = np.linalg.norm(unit_step)
        max_number_of_step = pathway_length/step
        unit_step = unit_step/pathway_length
        self.pathway = []
        print(unit_step)
        print(max_number_of_step)
        i = 0
        while i < max_number_of_step:
            self.pathway.append(origin + i*unit_step*step)
            i += 1
        self.pathway.append(target)
        for x in self.pathway:
            print(x)

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    linear_Pathway_Generation_node = linear_Pathway_Generation()  # Create an instance of the SimplePublisher
    try:
        rclpy.spin(linear_Pathway_Generation_node)
    except KeyboardInterrupt:
        pass
    finally:
        linear_Pathway_Generation_node.destroy_node()
    rclpy.shutdown()  # Shutdown the ROS2 Python client library
if __name__ == '__main__':
    main()  # Execute the main function
