from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Node chuyển CustomMsg sang PointCloud2
        Node(
            package='livox_to_laserscan',
            executable='livox_to_pointcloud',
            name='livox_to_pointcloud',
            output='screen',
            remappings=[
                ('/livox/lidar', '/livox/lidar'),  # đổi nếu topic khác
                ('/livox/points', '/livox/points')
            ]
        ),
        # Node chuyển PointCloud2 sang LaserScan
        Node(
            package='livox_to_laserscan',
            executable='pointcloud_to_scan',
            name='cloud_to_scan',
            output='screen',
            remappings=[
                ('/livox/points', '/livox/points'),  # đổi nếu topic khác
                ('/scan', '/scan')
            ]
        ),
    
        # Static TF từ base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_tf',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        )
    ])
