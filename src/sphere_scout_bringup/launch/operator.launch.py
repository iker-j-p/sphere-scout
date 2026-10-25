"""Start the operator side: target_localizer_node, thermal_viewer_node and RViz.

Owner: TODO(kickoff)

Does not start Gazebo, the bridge, the EKF or the detector (gazebo_world.launch.py and the
perception launch own those). Every node gets use_sim_time: true.

Launch args:
  rviz         (default true)  start RViz2
  rviz_config  (default sphere_scout_description/rviz/sphere_scout.rviz) layout saved from the
               RViz2 GUI. If the file does not exist RViz starts with its default layout.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz(context):
    config = LaunchConfiguration('rviz_config').perform(context)
    args, notes = [], []
    if os.path.isfile(config):
        args = ['-d', config]
    else:
        notes.append(LogInfo(msg=f'rviz_config not found ({config}); starting RViz with its default layout'))
    return notes + [Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=args, parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('sphere_scout_bringup'), 'config')
    default_rviz = os.path.join(get_package_share_directory('sphere_scout_description'),
                                'rviz', 'sphere_scout.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        Node(package='sphere_scout_ai', executable='target_localizer_node',
             name='target_localizer_node', output='screen',
             parameters=[os.path.join(cfg, 'target_localizer.yaml'), {'use_sim_time': True}]),
        Node(package='sphere_scout_ai', executable='thermal_viewer_node',
             name='thermal_viewer_node', output='screen',
             parameters=[os.path.join(cfg, 'thermal_viewer.yaml'), {'use_sim_time': True}]),
        OpaqueFunction(function=_rviz),
    ])
