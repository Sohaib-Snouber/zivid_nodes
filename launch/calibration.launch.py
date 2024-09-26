from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    calibration_node = Node(
        package='zivid_nodes',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
    )

    return LaunchDescription([
        calibration_node,
    ])
