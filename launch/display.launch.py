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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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

    robot_description = {
        "robot_description": robot_description_content}

    use_sim_time_param = {
        'use_sim_time': use_sim_time}

    zuuu_controller = PathJoinSubstitution(
        [
            FindPackageShare(
                "zuuu_description"),
            "config",
            "zuuu_controllers.yaml",
        ]
    )

    # Sam bot has use_sim_time at False on this one !
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description,
                    {"ignore_timestamp": True}]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, zuuu_controller, use_sim_time_param],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    # Call this to have 'ros2 control list_controllers' give :
    # joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        parameters=[use_sim_time_param],
        output='screen',
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[use_sim_time_param],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[use_sim_time_param],
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
                    use_sim_time_param]
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
        joint_state_publisher_node,
        # joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        spawn_entity,
        # controller_manager_node,
        # robot_localization_node,
        rviz_node
    ])
