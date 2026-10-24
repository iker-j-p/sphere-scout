"""Bring up the Sphere-Scout simulation.

Starts, in order (every node uses sim time):
  1. Gazebo with sphere_scout_gazebo/worlds/apartment.sdf (models dir on GZ_SIM_RESOURCE_PATH)
  2. robot_state_publisher with the processed robot xacro
  3. ros_gz_sim create: spawns sphere_scout at the entry pose
  4. ros_gz_bridge parameter_bridge with config/bridge.yaml
  5. robot_localization ekf_node with config/ekf.yaml
  6. RViz with sphere_scout_description/rviz/mission_control.rviz (optional)

Launch args:
  model     xacro file in sphere_scout_description/urdf. Defaults to the placeholder cube
            until sphere.urdf.xacro is finished.
  headless  true runs Gazebo server-only with headless rendering (sensors still work).
  rviz      false by default until mission_control.rviz is saved from RViz.
  x y z yaw deployment pose in the world frame. The default is on open ground outside the
            apartment's south wall, facing east along the rubble that spilled out of the
            collapse. The robot can't get into the ground floor from outside (closed walls and
            a 7 cm slab step), so it searches that debris. This pose is also the odom origin,
            so survivor coordinates are relative to it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gazebo(context):
    gazebo_share = get_package_share_directory('sphere_scout_gazebo')
    world = os.path.join(gazebo_share, 'worlds', 'apartment.sdf')
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    flags = '-s -r --headless-rendering' if headless else '-r'
    gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={'gz_args': f'{flags} {world}', 'on_exit_shutdown': 'true'}.items(),
    )]


def generate_launch_description():
    description_share = get_package_share_directory('sphere_scout_description')
    gazebo_share = get_package_share_directory('sphere_scout_gazebo')
    bringup_share = get_package_share_directory('sphere_scout_bringup')
    sim_time = {'use_sim_time': True}

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [description_share, 'urdf', LaunchConfiguration('model')])]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='placeholder_cube.urdf.xacro'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='5.0'),
        DeclareLaunchArgument('y', default_value='-2.5'),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gazebo_share, 'models')),
        OpaqueFunction(function=_gazebo),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time],
             output='screen'),

        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'sphere_scout',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
             parameters=[sim_time], output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(bringup_share, 'config', 'bridge.yaml')},
                         sim_time],
             output='screen'),

        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(bringup_share, 'config', 'ekf.yaml'), sim_time],
             output='screen'),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(description_share, 'rviz', 'mission_control.rviz')],
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
