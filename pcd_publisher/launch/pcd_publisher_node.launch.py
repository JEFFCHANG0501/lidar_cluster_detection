import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()
    
    package_name = 'pcd_publisher'

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'config.rviz'
    )

    pcd_publisher_node = launch_ros.actions.LifecycleNode(
        name='pcd_publisher_node',
        package=package_name,
        executable='pcd_publisher_node'
    )

    rviz2_node = launch_ros.actions.Node(
        name='rviz2_node',
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + rviz_config]
    )

    ld.add_action(pcd_publisher_node)
    ld.add_action(rviz2_node)

    return ld