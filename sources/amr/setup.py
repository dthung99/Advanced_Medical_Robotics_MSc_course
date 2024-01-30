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
            'ik_controller_null_space = amr.ik_controller_null_space:main',                    
            'hardware_interface = amr.hardware_interface:main',
            'simple_publisher = amr.simple_publisher:main',
            'controller_weighted_inverse_kinematic = amr.controller_weighted_inverse_kinematic:main',
            'control_pathway_generation = amr.control_pathway_generation:main',
            'ui_plotting = amr.ui_plotting:main',
            'ui_send_target = amr.ui_send_target:main',
            
        ],
    },
)
