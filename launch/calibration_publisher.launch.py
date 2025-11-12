from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


args = [
    {'name': 'camera_mount_frame',      'description': 'Name of the frame to which the camera is connected',            'default': ''},
    {'name': 'camera_frame',            'description': 'Name of the camera frame',                                      'default': ''},
    {'name': 'target_mount_frame',      'description': 'Name of the frame to which the calibration target is mounted',  'default': ''},
    {'name': 'target_frame',            'description': 'Name of the calibration target frame',                          'default': ''},
    {'name': 'camera_calibration_file', 'description': 'Path to the camera calibration file',                           'default': ''},
]


def declare_launch_arguments():
    """"""
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in args]


def generate_launch_description():
    """"""
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])


def create_camera_calibration_node(context,
                                   calibration: dict) -> ComposableNode:
    """"""
    return ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='camera_calibration',
        parameters=[{
            'frame_id': LaunchConfiguration('camera_mount_frame').perform(context),
            'child_frame_id': LaunchConfiguration('camera_frame').perform(context),
            'translation.x': calibration['x'],
            'translation.y': calibration['y'],
            'translation.z': calibration['z'],
            'rotation.x': calibration['qx'],
            'rotation.y': calibration['qy'],
            'rotation.z': calibration['qz'],
            'rotation.w': calibration['qw']
        }],
        extra_arguments=[{
            'use_intra_process_comms': True
        }]
    )


def create_target_calibration_node(context,
                                   calibration: dict) -> ComposableNode:
    """"""
    return ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='target_calibration',
        parameters=[{
            'frame_id': LaunchConfiguration('target_mount_frame').perform(context),
            'child_frame_id': LaunchConfiguration('target_frame').perform(context),
            'translation.x': calibration['x'],
            'translation.y': calibration['y'],
            'translation.z': calibration['z'],
            'rotation.x': calibration['qx'],
            'rotation.y': calibration['qy'],
            'rotation.z': calibration['qz'],
            'rotation.w': calibration['qw']
        }],
        extra_arguments=[{
            'use_intra_process_comms': True
        }]
    )


def launch(context, *args, **kwargs):
    """"""
    # Load the camera calibration YAML file
    calibration_file = LaunchConfiguration('camera_calibration_file')
    with open(calibration_file.perform(context), 'r') as f:
        calibration = yaml.safe_load(f)

    return [
        ComposableNodeContainer(
            name='transform_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
              create_camera_calibration_node(context, calibration['camera_mount_to_camera']),
              create_target_calibration_node(context, calibration['target_mount_to_target']),
            ],
            output='both',
        ),
    ]
