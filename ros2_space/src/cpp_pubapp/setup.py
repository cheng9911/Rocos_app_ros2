import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'cpp_pubapp'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name), glob('launch/*.py')),
        # Include model and simulation files
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sun',
    
    description='ROS 2 tutorial: Using URDF with robot_state_publisher',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = cpp_pubapp.state_publisher:main'
        ],
    },
)
