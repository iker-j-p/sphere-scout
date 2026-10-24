"""Start the perception pipeline (every node uses sim time).

Running:
  thermal_celsius_node  /thermal/image_raw (kelvin x 100) -> /thermal/celsius (32 x 24 floats, degC)
                        and the colour view for the Gazebo panel
  thermal_detector_node /thermal/image_raw -> /detected_target, by asking the human-detection model
                        service (config/thermal_detector.yaml). The service is a separate process
                        and is NOT started here: `python -m sphere_scout serve --port 8766` from
                        https://github.com/andrew2719/thermal_rescue_ai_model. Without it the
                        node warns and publishes nothing.

target_localizer_node and thermal_viewer_node are started by sphere_scout_bringup operator.launch.py,
not here (do not start the localizer from both launch files).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    config = os.path.join(get_package_share_directory('sphere_scout_ai'), 'config')
    return LaunchDescription([
        Node(package='sphere_scout_ai', executable='thermal_celsius_node',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='sphere_scout_ai', executable='thermal_detector_node',
             name='thermal_detector_node', output='screen',
             parameters=[os.path.join(config, 'thermal_detector.yaml'), {'use_sim_time': True}]),
    ])
