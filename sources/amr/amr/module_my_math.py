import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons, TextBox

class Kinematics:
    def __init__(self, joint_angle: np.ndarray, arm_length: np.ndarray = np.array([100, 100, 100]), angle_in_degree: bool = True) -> None:
        '''Results always in radians'''    
        # Change angle to radians if needed
        self.arm_1_position = None
        self.arm_2_position = None
        self.arm_3_position = None
        if angle_in_degree:
            joint_angle = np.deg2rad(joint_angle)
        # Store variables
        self.arm_length = arm_length
        self.joint_angle = joint_angle
        # Calculate position in task space
        self.Update_Data(joint_angle, angle_in_degree=False)
    
    def Update_Data(self, joint_angle: np.ndarray, angle_in_degree: bool = True) -> None:
        '''Update the current state'''
        # Change angle to radians if needed
        if angle_in_degree:
            joint_angle = np.deg2rad(joint_angle)
        # Store variables
        self.joint_angle = joint_angle
        arm_length = self.arm_length

        # Calculate position in task space
        self.arm_3_position = [np.cos(joint_angle[0])*arm_length[0] + np.cos(joint_angle[0]+joint_angle[1])*arm_length[1] + np.cos(joint_angle[0]+joint_angle[1]+joint_angle[2])*arm_length[2],
                               np.sin(joint_angle[0])*arm_length[0] + np.sin(joint_angle[0]+joint_angle[1])*arm_length[1] + np.sin(joint_angle[0]+joint_angle[1]+joint_angle[2])*arm_length[2],
                               0]
        self.arm_2_position = [np.cos(joint_angle[0])*arm_length[0] + np.cos(joint_angle[0]+joint_angle[1])*arm_length[1],
                               np.sin(joint_angle[0])*arm_length[0] + np.sin(joint_angle[0]+joint_angle[1])*arm_length[1], 
                               0]
        self.arm_1_position = [np.cos(joint_angle[0])*arm_length[0],
                               np.sin(joint_angle[0])*arm_length[0], 
                               0]
        return None

    def Jacobian(self) -> np.ndarray:
        '''Calculate 2x3 Jacobian'''
        x = self.joint_angle
        l = self.arm_length
        ''' Formula for Jacobian
        [[-r1s1-r2s12-r3s123, -r2s12-r3s123, -r3s123],
        [+r1c1+r2c12+r3c123, +r2c12+r3c123, +r3c123]] 
        '''
        return np.array([[-l[0]*np.sin(x[0])-l[1]*np.sin(x[0]+x[1])-l[2]*np.sin(x[0]+x[1]+x[2]), -l[1]*np.sin(x[0]+x[1])-l[2]*np.sin(x[0]+x[1]+x[2]),-l[2]*np.sin(x[0]+x[1]+x[2])],
                         [+l[0]*np.cos(x[0])+l[1]*np.cos(x[0]+x[1])+l[2]*np.cos(x[0]+x[1]+x[2]), +l[1]*np.cos(x[0]+x[1])+l[2]*np.cos(x[0]+x[1]+x[2]),+l[2]*np.cos(x[0]+x[1]+x[2])]])
    
    def get_Weighted_Velocity_needed_for_Target(self,
                                                target,
                                                d_error_clamping: float = np.finfo(float).max,
                                                weight_vector: np.ndarray = np.array([1, 1, 1]),
                                                k_damped_factor: float = 0,
                                                ) -> np.ndarray:
        '''Calculate the joint velocity to get to target with error clamping, weight matrix, and jacobian damping'''
        jacobian = self.Jacobian()
        # weight_matrix = np.diag(weight_vector)
        inverse_weight_matrix = np.diag(1/weight_vector)
        matrix_a = np.matmul(inverse_weight_matrix, jacobian.T)
        matrix_b = np.linalg.inv(np.matmul(np.matmul(jacobian,inverse_weight_matrix), jacobian.T) + k_damped_factor**2*np.eye(2))
        # Calculate the pseudo inverse: J^T*inv(J*JT)
        pseudo_inv = np.matmul(matrix_a, matrix_b)
        task_space_vel = self.error_clamping(target - self.arm_3_position, d_error_clamping)
        task_space_vel = task_space_vel[0:2]
        # Calculate the target velocity
        joint_space_vel = np.matmul(pseudo_inv,task_space_vel.reshape(-1,1)).flatten()
        return joint_space_vel
    
    def error_clamping(self, input_vector: np.ndarray, d_error_clamping: float):
        length = np.linalg.norm(input_vector)
        if length > d_error_clamping:
            return input_vector/length*d_error_clamping
        else:
            return input_vector




# class Forward_Kinematics:
#     def __init__(self, joint_angle: np.ndarray, arm_length: np.ndarray = np.array([100, 100, 100]), angle_in_degree: bool = True) -> None:
#         '''Results always in radians'''    
#         # Change angle to radians if needed
#         self.arm_1_position = None
#         self.arm_2_position = None
#         self.arm_3_position = None
#         if angle_in_degree:
#             joint_angle = np.deg2rad(joint_angle)
#         # Store variables
#         self.arm_length = arm_length
#         self.joint_angle = joint_angle
#         # Calculate position in task space
#         self.Update_Data(joint_angle, angle_in_degree=False)
#         # Add figures and axes
#         self.fig = plt.figure(num='Team 5')
#         self.ax = self.fig.add_subplot([0.1, 0.05, 0.4, 0.8])
#         # Add widgets
#         self.button_left_1 = Button(self.fig.add_axes([0.35, 0.9, 0.1, 0.05]), "Send!!")
#         self.textbox_left_1 = TextBox(self.fig.add_axes([0.05, 0.9, 0.1, 0.05]), "")
#         self.textbox_left_2 = TextBox(self.fig.add_axes([0.20, 0.9, 0.1, 0.05]), "")

#         checkbox_ax = self.fig.add_axes([0.1, 0.5, 0.1, 0.1])
#         checkbox_labels = ['Option 1', 'Option 2', 'Option 3']
#         self.checkbox = CheckButtons(checkbox_ax, checkbox_labels)

#         # Plot working space"
#         theta = np.linspace(-np.pi/2, np.pi/2, 100)
#         x = 300*np.cos(theta)
#         y = 300*np.sin(theta)
#         self.ax.plot(x, y, color = "black", linewidth=0.5)        
#         # self.ax.set_xlim(0, 350)
#         # self.ax.set_ylim(-350, 350)
#         self.ax.axis('equal')
#         self.arm_1, = self.ax.plot([0, self.arm_1_position[0]], [0, self.arm_1_position[1]], color = "red")
#         self.arm_2, = self.ax.plot([self.arm_1_position[0], self.arm_2_position[0]], [self.arm_1_position[1], self.arm_2_position[1]], color = "green")
#         self.arm_3, = self.ax.plot([self.arm_2_position[0], self.arm_3_position[0]], [self.arm_2_position[1], self.arm_3_position[1]], color = "blue")
    
#     def Jacobian(self) -> np.ndarray:
#         '''Calculate 2x3 Jacobian'''
#         x = self.joint_angle
#         l = self.arm_length
#         ''' Formula for Jacobian
#         [[-r1s1-r2s12-r3s123, -r2s12-r3s123, -r3s123],
#         [+r1c1+r2c12+r3c123, +r2c12+r3c123, +r3c123]] 
#         '''
#         return np.array([[-l[0]*np.sin(x[0])-l[1]*np.sin(x[0]+x[1])-l[2]*np.sin(x[0]+x[1]+x[2]), -l[1]*np.sin(x[0]+x[1])-l[2]*np.sin(x[0]+x[1]+x[2]),-l[2]*np.sin(x[0]+x[1]+x[2])],
#                          [+l[0]*np.cos(x[0])+l[1]*np.cos(x[0]+x[1])+l[2]*np.cos(x[0]+x[1]+x[2]), +l[1]*np.cos(x[0]+x[1])+l[2]*np.cos(x[0]+x[1]+x[2]),+l[2]*np.cos(x[0]+x[1]+x[2])]])
    
#     def Update_Data(self, joint_angle: np.ndarray, angle_in_degree: bool = True) -> None:
#         '''Update the current state'''
#         # Change angle to radians if needed
#         if angle_in_degree:
#             joint_angle = np.deg2rad(joint_angle)
#         # Store variables
#         self.joint_angle = joint_angle
#         arm_length = self.arm_length

#         # Calculate position in task space
#         self.arm_3_position = [np.cos(joint_angle[0])*arm_length[0] + np.cos(joint_angle[0]+joint_angle[1])*arm_length[1] + np.cos(joint_angle[0]+joint_angle[1]+joint_angle[2])*arm_length[2],
#                                np.sin(joint_angle[0])*arm_length[0] + np.sin(joint_angle[0]+joint_angle[1])*arm_length[1] + np.sin(joint_angle[0]+joint_angle[1]+joint_angle[2])*arm_length[2],
#                                0]
#         self.arm_2_position = [np.cos(joint_angle[0])*arm_length[0] + np.cos(joint_angle[0]+joint_angle[1])*arm_length[1],
#                                np.sin(joint_angle[0])*arm_length[0] + np.sin(joint_angle[0]+joint_angle[1])*arm_length[1], 
#                                0]
#         self.arm_1_position = [np.cos(joint_angle[0])*arm_length[0],
#                                np.sin(joint_angle[0])*arm_length[0], 
#                                0]
#         return None
    
#     def Update_Plot(self) -> None:
#         # Plot
#         self.arm_1.set_data([0, self.arm_1_position[0]], [0, self.arm_1_position[1]])
#         self.arm_2.set_data([self.arm_1_position[0], self.arm_2_position[0]], [self.arm_1_position[1], self.arm_2_position[1]])
#         self.arm_3.set_data([self.arm_2_position[0], self.arm_3_position[0]], [self.arm_2_position[1], self.arm_3_position[1]])
#         self.fig.canvas.draw()
#         return
