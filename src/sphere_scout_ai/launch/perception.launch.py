"""Start the perception pipeline (every node uses sim time).

Running now:
  thermal_celsius_node  /thermal/image_raw (kelvin x 100) -> /thermal/celsius (32 x 24 floats, degC)

TODO: start thermal_detector_node and target_localizer_node once they are implemented, and pass
      ground_truth_file = sphere_scout_gazebo/config/ground_truth.yaml to the localizer.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='sphere_scout_ai', executable='thermal_celsius_node',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
