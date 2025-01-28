import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ilumo_startup'),
        'config',
        'params.yaml'
        )

    stereo_camera_node = Node(
        package='ilumo_sensors',
        executable='stereo_camera_publisher',
        name='stereo_camera',
        parameter=[config]
    )

    lidar_node = Node(
        package='ilumo_sensors',
        executable='lidar_publisher',
        name='lidar',
        parameter=[config]
    )

    thermal_camera_node = Node(
        package='ilumo_sensors',
        executable='thermal_camera_publisher',
        name='thermal_camera',
        parameter=[config]
    )

    pwm_controller_node = Node(
        package='ilumo_control',
        executable='pwm_controller',
        name='pwm_controller',
        parameter=[config]
    )

    kinematics_node = Node(
        package='ilumo_description',
        executable='ilumo_kinematics',
        name='kinematics',
        parameter=[config]
    )

    return LaunchDescription([
        stereo_camera_node,
        lidar_node,
        thermal_camera_node,
        pwm_controller_node,
        kinematics_node
    ])