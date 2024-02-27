from setuptools import find_packages, setup

package_name = 'amr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ll',
    maintainer_email='lukas.lindenroth@kcl.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Tsubscriber = amr.Tsubscriber:main',
            'Tpublisher = amr.Tpublisher:main', 
            'Thardware_interface = amr.Thardware_interface:main',
            'hardware_interface = amr.hardware_interface:main',
            'hardware_interface_motor_only = amr.hardware_interface_motor_only:main',            
            'ui_plotting = amr.ui_plotting:main',
            'ui_send_target_for_pathway = amr.ui_send_target_for_pathway:main',
            'ui_send_target = amr.ui_send_target:main',
            'ui_send_target_for_one_motor = amr.ui_send_target_for_one_motor:main',
            'command_pathway_generation = amr.command_pathway_generation:main',
            'speed_control_with_jacobian = amr.speed_control_with_jacobian:main',
            'speed_control_with_jacobian_own_micro = amr.speed_control_with_jacobian_own_micro:main',
            'speed_control_in_null_space = amr.speed_control_in_null_space:main',
            'angle_control_with_jacobian_solver = amr.angle_control_with_jacobian_solver:main',
            'micro_pid_angle_control = amr.micro_pid_angle_control:main',
            'micro_pid_velocity_control = amr.micro_pid_velocity_control:main',
            'micro_pid_velocity_control_1 = amr.micro_pid_velocity_control_1:main',
            'micro_pid_velocity_control_2 = amr.micro_pid_velocity_control_2:main',
            'micro_pid_velocity_control_3 = amr.micro_pid_velocity_control_3:main',
            'micro_pid_velocity_control_3_motors = amr.micro_pid_velocity_control_3_motors:main',
            'micro_pid_velocity_control_combined_publisher = amr.micro_pid_velocity_control_combined_publisher:main',
            'test_node = amr.test_node:main',   
            'contact_force = amr.contact_force:main',

            'force_end_effector_control = amr.force_end_effector_control:main',
        ],
    },
)
