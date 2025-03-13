from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            #prefix=['gdbserver localhost:3000'],
            package='krabi_can_broker',
            namespace='krabi_ns',
            executable='motor_broker',
            name='motor_broker'
        )
    ])