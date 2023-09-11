import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeyoung',
    maintainer_email='jalim@ethz.ch',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'offboard_mission = px4_offboard.offboard_mission:main',
                'offboard_nominal = px4_offboard.offboard_nominal:main',
                'visualizer = px4_offboard.visualizer:main',
                'stream_mocap = px4_offboard.stream_mocap:main',
                'offboard_control_exp = px4_offboard.offboard_control_exp:main',
                'offboard_mission_exp = px4_offboard.offboard_mission_exp:main',
                'offboard_nominal_exp = px4_offboard.offboard_nominal_exp:main',
                'offboard_tsunomi_1_exp = px4_offboard.offboard_tsunomi_1_exp:main',
                'offboard_tsunomi_2_exp = px4_offboard.offboard_tsunomi_2_exp:main',
                'offboard_tsunomi_3_exp = px4_offboard.offboard_tsunomi_3_exp:main',
                'offboard_tsunomi_4_exp = px4_offboard.offboard_tsunomi_4_exp:main',
                'offboard_tsunomi_5_exp = px4_offboard.offboard_tsunomi_5_exp:main',
                'offboard_tsunomi_6_exp = px4_offboard.offboard_tsunomi_6_exp:main',
                'offboard_tsunomi_7_exp = px4_offboard.offboard_tsunomi_7_exp:main',
                'offboard_tsunomi_8_exp = px4_offboard.offboard_tsunomi_8_exp:main',
                'offboard_tsunomi_9_exp = px4_offboard.offboard_tsunomi_9_exp:main',
        ],
    },
)
