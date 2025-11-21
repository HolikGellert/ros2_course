from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_course'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_user',
    maintainer_email='ros_user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = ros2_course.hello:main',
            'talker = ros2_course.talker:main',
            'listener = ros2_course.listener:main',
            'turtlesim_controller = ros2_course.turtlesim_controller:main',
            'psm_grasp = ros2_course.psm_grasp:main',
            'dummy_marker = ros2_course.dummy_marker:main',
            'topic_latcher = ros2_course.topic_latcher:main',
            'ur_controller = ros2_course.ur_controller:main',
            'grasp_server = ros2_course.grasp_server:main',
            'grasp_client = ros2_course.grasp_client:main',
            'wall_follower = ros2_course.wall_follower:main',
        ],
    },
)
