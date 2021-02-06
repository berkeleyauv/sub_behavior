from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='behavior_tree',
            namespace='',
            executable='bt_engine',
            # Do not declare a node name otherwise it messes with the action node names and will result in
            # duplicate nodes!
            parameters=[
                {'bt_file_path': os.path.join(get_package_share_directory('sub_behavior'), 'trees/example.xml')},
                {'plugins': ['move_distance_bt_node', 'random_failure_bt_node', 'random_displacement_bt_node']}
            ]
        ),
    ])
