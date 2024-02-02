import numpy as np
import matplotlib.pyplot as plt

class Forward_Kinematics:
    def __init__(self, joint_angle: np.ndarray, arm_length: np.ndarray = np.array([100, 100, 100]), angle_in_degree: bool = True) -> None:
        '''Results always in radians'''    
        # Change angle to radians if needed
        if angle_in_degree:
            joint_angle = np.deg2rad(joint_angle)
        self.joint_angle = joint_angle
        self.arm_length = arm_length
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



    def Jacobian(self) -> None:
        x = self.joint_angle
        l = self.arm_length
        ''' Formula for Jacobian
        [[-r1s1-r2s12-r3s123, -r2s12-r3s123, -r3s123],
        [+r1c1+r2c12+r3c123, +r2c12+r3c123, +r3c123]] 
        '''
        return np.array([[-l[0]*np.sin(x[0])-l[1]*np.sin(x[0]+x[1])-l[2]*np.sin(x[0]+x[1]+x[2]), -l[1]*np.sin(x[0]+x[1])-l[2]*np.sin(x[0]+x[1]+x[2]),-l[2]*np.sin(x[0]+x[1]+x[2])],
                         [+l[0]*np.cos(x[0])+l[1]*np.cos(x[0]+x[1])+l[2]*np.cos(x[0]+x[1]+x[2]), +l[1]*np.cos(x[0]+x[1])+l[2]*np.cos(x[0]+x[1]+x[2]),+l[2]*np.cos(x[0]+x[1]+x[2])]])

    
                                   