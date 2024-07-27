import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'other_things'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_send_target_for_pathway = other_things.ui_send_target_for_pathway:main',
            'enter_target_for_pathway = other_things.enter_target_for_pathway:main',

            'command_pathway_generation = other_things.command_pathway_generation:main',

            'speed_control_with_jacobian_own_micro = other_things.speed_control_with_jacobian_own_micro:main',

            'micro_pid_velocity_control_3_motors = other_things.micro_pid_velocity_control_3_motors:main',
            'micro_pid_velocity_control = other_things.micro_pid_velocity_control:main',

            'test_node_other_things = other_things.test_node_other_things:main',
            'contact_force = other_things.contact_force:main',
            'force_end_effector_control = other_things.force_end_effector_control:main'
        ],
    },
)