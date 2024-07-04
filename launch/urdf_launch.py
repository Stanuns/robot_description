from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('robot_description')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'diff_robot.urdf.xacro')

    camera_model = LaunchConfiguration('camera_model', default='OAK-D-LITE').perform(context)
    tf_prefix = LaunchConfiguration('tf_prefix', default='oak').perform(context)
    base_frame = LaunchConfiguration('base_frame', default='oak-d_frame').perform(context)
    parent_frame = LaunchConfiguration('parent_frame', default='panel_link').perform(context)
    cam_pos_x = LaunchConfiguration('cam_pos_x', default='0.0').perform(context)
    cam_pos_y = LaunchConfiguration('cam_pos_y', default='0.0').perform(context)
    cam_pos_z = LaunchConfiguration('cam_pos_z', default='0.0').perform(context)
    cam_roll = LaunchConfiguration('cam_roll', default='1.5708').perform(context)
    cam_pitch = LaunchConfiguration('cam_pitch', default='0.0').perform(context)
    cam_yaw = LaunchConfiguration('cam_yaw', default='1.5708').perform(context)
    namespace = LaunchConfiguration('namespace', default='').perform(context)
    use_composition = LaunchConfiguration('use_composition_a', default='false')
    use_rviz = LaunchConfiguration("use_rviz", default='false').perform(context)

    name = tf_prefix

    return [
        Node(
            package='robot_state_publisher',
            # condition=UnlessCondition(use_composition),
            executable='robot_state_publisher',
            # name=name+'_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', tf_prefix, ' ',
                    'camera_model:=', camera_model, ' ',
                    'base_frame:=', base_frame, ' ',
                    'parent_frame:=', parent_frame, ' ',
                    'cam_pos_x:=', cam_pos_x, ' ',
                    'cam_pos_y:=', cam_pos_y, ' ',
                    'cam_pos_z:=', cam_pos_z, ' ',
                    'cam_roll:=', cam_roll, ' ',
                    'cam_pitch:=', cam_pitch, ' ',
                    'cam_yaw:=', cam_yaw
                ])}]
            ),
        Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        Node(
            package='joint_state_publisher',
            # condition=UnlessCondition(use_composition),
            executable='joint_state_publisher',
            # name=name+'_joint_publisher',
            namespace=namespace,
            parameters=[{'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', tf_prefix, ' ',
                    'camera_model:=', camera_model, ' ',
                    'base_frame:=', base_frame, ' ',
                    'parent_frame:=', parent_frame, ' ',
                    'cam_pos_x:=', cam_pos_x, ' ',
                    'cam_pos_y:=', cam_pos_y, ' ',
                    'cam_pos_z:=', cam_pos_z, ' ',
                    'cam_roll:=', cam_roll, ' ',
                    'cam_pitch:=', cam_pitch, ' ',
                    'cam_yaw:=', cam_yaw
                ])}]
            ),
            # LoadComposableNodes(
            # target_container=name+"_container",
            # condition=IfCondition(use_composition),
            # composable_node_descriptions=[
            #     ComposableNode(
            #         package='robot_state_publisher',
            #         plugin='robot_state_publisher::RobotStatePublisher',
            #         name=name+'_state_publisher',
            #         parameters=[{'robot_description': Command(
            #             [
            #                 'xacro', ' ', xacro_path, ' ',
            #                 'camera_name:=', tf_prefix, ' ',
            #                 'camera_model:=', camera_model, ' ',
            #                 'base_frame:=', base_frame, ' ',
            #                 'parent_frame:=', parent_frame, ' ',
            #                 'cam_pos_x:=', cam_pos_x, ' ',
            #                 'cam_pos_y:=', cam_pos_y, ' ',
            #                 'cam_pos_z:=', cam_pos_z, ' ',
            #                 'cam_roll:=', cam_roll, ' ',
            #                 'cam_pitch:=', cam_pitch, ' ',
            #                 'cam_yaw:=', cam_yaw
            #             ])}]
            #     )])
            ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'name',
            default_value='',
            description='Specifies the namespace of the robot state publisher node. Default value will be ""'),
        DeclareLaunchArgument(
            'camera_model',
            default_value='OAK-D-LITE',
            description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.'),

        DeclareLaunchArgument(
            'tf_prefix',
            default_value='oak',
            description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),

        DeclareLaunchArgument(
            'base_frame',
            default_value='oak-d_frame',
            description='Name of the base link.'),

        DeclareLaunchArgument(
            'parent_frame',
            default_value='panel_link',
            description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.'),

        DeclareLaunchArgument(
            'cam_pos_x',
            default_value='0.21',
            description='Position X of the camera with respect to the base frame.'),

        DeclareLaunchArgument(
            'cam_pos_y',
            default_value='-0.105',
            description='Position Y of the camera with respect to the base frame.'),

        DeclareLaunchArgument(
            'cam_pos_z',
            default_value='0.0',
            description='Position Z of the camera with respect to the base frame.'),

        DeclareLaunchArgument(
            'cam_roll',
            default_value='0.0',
            description='Roll orientation of the camera with respect to the base frame.'),

        DeclareLaunchArgument(
            'cam_pitch',
            default_value='0.0',
            description='Pitch orientation of the camera with respect to the base frame.'),

        DeclareLaunchArgument(
            'cam_yaw',
            default_value='0.0',
            description='Yaw orientation of the camera with respect to the base frame.'),
        DeclareLaunchArgument(
            'use_composition_a',
            default_value='false',
            description='Use composition to start the robot_state_publisher node. Default value will be false')
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
