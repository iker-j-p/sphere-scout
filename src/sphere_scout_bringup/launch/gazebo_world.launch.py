"""Bring up the Sphere-Scout simulation.

Owner: TODO(kickoff)

TODO, in order (every node gets use_sim_time: true):
  1. Append sphere_scout_gazebo/models to GZ_SIM_RESOURCE_PATH.
  2. Gazebo via ros_gz_sim gz_sim.launch.py with sphere_scout_gazebo/worlds/apartment.sdf.
     Launch arg `headless` (default false): true -> '-s -r --headless-rendering', else '-r'.
  3. robot_state_publisher with the processed sphere.urdf.xacro.
  4. Spawn with ros_gz_sim `create`: -topic robot_description -name sphere_scout,
     at the entry pose (launch args; default values TODO(kickoff)).
  5. ros_gz_bridge parameter_bridge with config/bridge.yaml.
  6. robot_localization ekf_node with config/ekf.yaml.
  7. Launch arg `rviz` (default true) opens sphere_scout_description/rviz/mission_control.rviz.
"""

from launch import LaunchDescription


def generate_launch_description():
    # TODO: implement
    return LaunchDescription([])
