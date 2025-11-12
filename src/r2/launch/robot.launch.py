from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_description_path = get_package_share_path('r2')

    urdf_path = os.path.join(robot_description_path, 'urdf', 'robot_description.urdf.xacro')
    rviz_config_path = os.path.join(robot_description_path, 'rviz', 'mecanum.rviz')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_controllers = os.path.join(robot_description_path, 'config', 'mecanum_robot.yaml')

    # --- Robot Description ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }],
    )

    # --- ROS2 Control (with TF remaps) ---
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            robot_controllers
        ],
        remappings=[
            ('/mecanum_drive_controller/tf_odometry', '/tf'),
            ('/mecanum_drive_controller/tf_static', '/tf_static'),
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller'],
        output='screen',
    )

    # --- RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])
