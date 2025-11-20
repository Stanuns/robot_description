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
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    #机器人模型
    robot_des_path = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'akm_robot_sim.xacro')
    robot_des = Command(['xacro ', robot_des_path])
    # doc = xacro.parse(open(robot_description_path))
    # xacro.process_doc(doc)
    robot_description = {'robot_description': robot_des}

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


    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    return ld

    

