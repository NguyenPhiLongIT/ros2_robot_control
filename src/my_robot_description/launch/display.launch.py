from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('my_robot_description'),
                             'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'rviz', 'urdf_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    gazebo_launch = os.path.join(
        get_package_share_path('my_robot_bringup'),
        'launch',
        'my_robot_gazebo.launch.xml'
    )

    gazebo_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(gazebo_launch)
    )

    keyboard_gui_node = Node(
        package="my_robot_control",
        executable="keyboard_gui",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        gazebo_node,
        keyboard_gui_node
    ])