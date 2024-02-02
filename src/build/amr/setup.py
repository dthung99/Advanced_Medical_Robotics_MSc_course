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
            'hardware_interface = amr.hardware_interface:main',
            'ui_plotting = amr.ui_plotting:main',
            'ui_send_target_for_pathway = amr.ui_send_target_for_pathway:main',
            'ui_send_target = amr.ui_send_target:main',
            'command_pathway_generation = amr.command_pathway_generation:main',
            'speed_control_with_jacobian = amr.speed_control_with_jacobian:main',
            'angle_control_with_jacobian_solver = amr.angle_control_with_jacobian_solver:main',
            'test_node = amr.test_node:main',          
        ],
    },
)
