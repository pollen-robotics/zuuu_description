import launch
import launch_ros
import os

from launch import LaunchDescription
from launch import event_handlers
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='zuuu_description').find('zuuu_description')
    default_model_path = os.path.join(
        pkg_share, 'src/description/zuuu_description.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'world/my_world.sdf')

    # robot_description_content = Command(
    #     ['xacro ', LaunchConfiguration('model')])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('zuuu_description'),
                 'src', 'description', 'zuuu_description.urdf.xacro']
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    zuuu_controller = PathJoinSubstitution(
        [
            FindPackageShare(
                "zuuu_description"),
            "config",
            "zuuu_controllers.yaml",
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, zuuu_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [PathJoinSubstitution(
    #             [launch_ros.substitutions.FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
    #     ),
    #     launch_arguments={'verbose': 'true'}.items(),
    # )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'zuuu', '-topic',
                   'robot_description'],
        output='screen'
    )
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'controllers_file',
            default_value=['zuuu_controllers.yaml'],
            description='YAML file with the controllers configuration.',
        ),
        DeclareLaunchArgument(name='gui', default_value='True',
                              description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                              description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                              description='Flag to enable use_sim_time'),

        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        spawn_entity,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node
    ])
