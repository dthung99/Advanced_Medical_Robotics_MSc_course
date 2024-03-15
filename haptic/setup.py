from setuptools import find_packages, setup

package_name = 'haptic'

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
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_interface = haptic.hardware_interface:main',

            'force_end_effector_control = haptic.force_end_effector_control:main',

            'test_node = haptic.test_node:main',
            'test_node1 = haptic.test_node1:main',
            'test_node2 = haptic.test_node2:main',
            'test_node3 = haptic.test_node3:main',
            'test_node4 = haptic.test_node4:main',
            'test_node5 = haptic.test_node5:main',
            'test_node6 = haptic.test_node6:main',
        ],
    },
)
