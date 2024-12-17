from setuptools import find_packages, setup

package_name = 'distance_calculator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+ '/launch', ['launch/realsense_launch.py']),
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='kalyani-robotics',
    maintainer_email='sanskar.jadhav@kalyani.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_calculator = distance_calculator.distance_calculator:main',
            'click_to_depth = distance_calculator.click_to_depth:main',
            'CenterPointFinder = distance_calculator.CenterPointFinder:main',
            'point_cloud_centre = distance_calculator.point_cloud_centre:main',
            'nearest = distance_calculator.nearest:main',
            'yolo_ros = distance_calculator.yolo_ros:main',
            'pointcloud_to_laser = distance_calculator.pointcloud_to_laser:main',
            'rviz_marker_pub = distance_calculator.rviz_marker_pub:main',
            'robot_pose = distance_calculator.robot_pose:main',
            'speak = distance_calculator.speak:main',
            'pxl_map_dist_angl = distance_calculator.pxl_map_dist_angl:main'
        ],
    },
)
