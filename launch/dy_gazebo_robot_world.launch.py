import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions.append_environment_variable import AppendEnvironmentVariable

def generate_launch_description():
    robot_name_in_model = 'dy_robot'
    package_name = 'robot_description'
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    pkg_share = get_package_share_directory(package_name)
    gazebo_world_path = os.path.join(pkg_share, 'world/carto_test.world')

    # Start Gazebo server - world模型
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        output='screen')

    #机器人模型
    robot_des_path = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'akm_robot_sim.xacro')
    robot_des = Command(['xacro ', robot_des_path])
    # doc = xacro.parse(open(robot_description_path))
    # xacro.process_doc(doc)
    robot_description = {'robot_description': robot_des}

    # gazebo_ros_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
    # )
    # gazebo_client_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')),
    # )
    # Set Gazebo plugin path
    # append_enviroment = AppendEnvironmentVariable(
    #     'GAZEBO_PLUGIN_PATH',
    #     os.path.join(os.path.join(get_package_share_directory('robot_description'), 'meshes'))
    # )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {
            'use_sim_time': use_sim_time
        }]
    )
    # Robot Joint Publisher
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description, {
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_diff_drive_base_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'diff_drive_base_controller'],
    #     output='screen'
    # )


    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  
                   '-topic', robot_description,
                   '-x', '0', 
                   '-y', '0',
                   '-z', '0',
                   '-Y', '0'],
        output='screen')

    # 注意启动顺序
    joint_state_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[start_joint_state_publisher_cmd],
        )
    )


    ld = LaunchDescription()

    # Set environment variables
    # ld.add_action(append_enviroment)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_gazebo_cmd)
    # ld.add_action(gazebo_client_launch)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_after_spawn_entity)
    ld.add_action(spawn_entity_cmd)
    # ld.add_action(gazebo_ros_node)
    return ld

    

