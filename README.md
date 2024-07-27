# Advanced Medical Robotics Project

This is my project for the 'Advanced Medical Robotics' module of my MSc at King's College London, whose main focus is about control systems. I create this repository long after the project finished. The codes are somwhat commented, however, the directory structure is not really well formatted.

# Aim and Objectives:

Building a haptic device with a 3 degree-of-freedom planar robot. 

# Method:

I solve this problem by building two systems:

- A haptics rendering system that generating a MxNxD tensor that store the environment (haptics information), where MxN is the workspace, and D is the information dimension that store different parameters about the environment that we want to simulate.

- Using the parameters of the environment at point MxN, the robot simulate the force at that point by controlling the motor torque through the following formula:

$\tau = J^T \cdot F$

- Additionally, I did identify the dynamic and friction models of the robot to aid the system.

# Result:

I got 8.3/10 for my report.

# Note

This section indicates some of the path and the purpose of the codes provided there. Because the codes are designed as a distributed system, most of the codes cannot be run alone.

- [generate_robot_environment\create_environment_main_new.py](generate_robot_environment\create_environment_main_new.py): This script is runable by itself. It is used for generating a tensor that store the environment parameters as mentioned.

- [ROS2_package\haptic\haptic](ROS2_package\haptic\haptic): This is where the main codes for controlling the robot reside.

    + hardware_interface.py is for communicating with the microcontroller, and needed to be modified for different setup.

    + force_end_effector_control.py is for controlling the robot to generate the required force, which is the main purpose of the project.

    + UI.py is for visualizing the robot.

    + module_my_math.py is my implementation of the importance Maths used in the project.

- [ROS2_package\other_things\other_things](ROS2_package\other_things\other_things): This is where the codes used for a Jacobian-based task space controller reside. It is an auxiliary objective of the project. Luckily, the [ROS2_package\other_things\launch\jacobian_controller_launch.py](ROS2_package\other_things\launch\jacobian_controller_launch.py) helps launch the neccesary nodes.

