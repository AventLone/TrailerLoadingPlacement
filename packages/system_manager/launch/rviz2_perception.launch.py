"""Run localization, placement perception and optional robot/RViz publishers."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def apply_lidar_extrinsics(robot_description, config_path):
    """Override only the fixed mount joint; robot_state_publisher owns its TF."""
    config = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    if not isinstance(config, dict) or not isinstance(config.get('lidar_extrinsics'), dict):
        raise ValueError(f'{config_path}: expected a lidar_extrinsics mapping')
    mount = config['lidar_extrinsics']
    names = {}
    for key in ('joint_name', 'parent_frame', 'child_frame'):
        value = mount.get(key)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'{config_path}: {key} must be a nonempty frame/joint name')
        names[key] = value

    def vector(section, axes):
        values = mount.get(section)
        if not isinstance(values, dict):
            raise ValueError(f'{config_path}: {section} must be a mapping')
        result = []
        for axis in axes:
            value = values.get(axis)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{config_path}: {section}.{axis} must be a finite number')
            result.append(format(value, '.17g'))
        return ' '.join(result)

    xyz = vector('translation', ('x', 'y', 'z'))
    rpy = vector('rotation_rpy', ('roll', 'pitch', 'yaw'))
    root = ET.fromstring(robot_description)
    joints = [joint for joint in root.findall('joint')
              if joint.get('name') == names['joint_name']]
    if len(joints) != 1 or joints[0].get('type') != 'fixed':
        raise ValueError(f'{config_path}: expected one fixed URDF joint {names["joint_name"]!r}')
    joint = joints[0]
    for tag, key in (('parent', 'parent_frame'), ('child', 'child_frame')):
        element = joint.find(tag)
        if element is None or element.get('link') != names[key]:
            raise ValueError(f'{config_path}: {key} does not match URDF joint {names["joint_name"]!r}')
    origin = joint.find('origin')
    if origin is None:
        origin = ET.SubElement(joint, 'origin')
    origin.set('xyz', xyz)
    origin.set('rpy', rpy)
    return ET.tostring(root, encoding='unicode')


def robot_publishers(context):
    if LaunchConfiguration('publish_robot').perform(context).lower() != 'true':
        if LaunchConfiguration('publish_static_tf').perform(context).lower() == 'true':
            arguments = []
            for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                arguments.extend(['--' + axis, LaunchConfiguration('lidar_' + axis)])
            return [Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='lidar_mount_tf', output='screen',
                arguments=arguments + ['--frame-id', 'LOLA', '--child-frame-id', 'JT128'],
                parameters=[{'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool)}],
            )]
        return []
    model_path = Path(LaunchConfiguration('robot_description_file').perform(context))
    robot_description = model_path.read_text(encoding='utf-8')
    extrinsics_file = LaunchConfiguration('lidar_extrinsics_file').perform(context)
    if extrinsics_file:
        robot_description = apply_lidar_extrinsics(robot_description, extrinsics_file)
    parameters = [{
        'robot_description': ParameterValue(robot_description, value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }]
    return [
        LogInfo(msg=f'Robot model: {model_path}; LiDAR extrinsics: {extrinsics_file or "URDF origin"}'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=parameters),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             output='screen', parameters=parameters + [{'source_list': ['/lola/joint_states']}]),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory('system_manager'))
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('publish_robot', default_value='true',
                              description='Load the supplied forklift URDF and publish its TF tree.'),
        DeclareLaunchArgument('publish_static_tf', default_value='false',
                              description='Publish a temporary mount TF when publish_robot is false.'),
        *[DeclareLaunchArgument('lidar_' + axis, default_value='0.0')
          for axis in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')],
        DeclareLaunchArgument('robot_description_file',
                              default_value=str(package_share / 'urdf' / 'lola.urdf')),
        DeclareLaunchArgument('lidar_extrinsics_file',
                              default_value=str(package_share / 'config' / 'param' / 'lidar_extrinsics.yaml'),
                              description='Mount YAML applied to the URDF; empty uses the original URDF origin.'),
        DeclareLaunchArgument('rviz_config',
                              default_value=str(package_share / 'config' / 'rviz' / 'perception.rviz')),
        DeclareLaunchArgument('points_topic', default_value='/sim_scan'),
        OpaqueFunction(function=robot_publishers),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')), parameters=[clock],
             arguments=['-d', LaunchConfiguration('rviz_config')]),
        Node(package='localization', executable='localization', namespace='localization',
             output='screen', parameters=[clock],
             remappings=[('/sim_scan', LaunchConfiguration('points_topic'))]),
        Node(package='free_space_detection', executable='free_space_detection',
             namespace='free_space_detection', output='screen', parameters=[clock]),
    ])
