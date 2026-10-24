"""target_localizer_node: triangulates the survivor from thermal detections.

Owner: TODO(kickoff)

TODO: rclpy node `target_localizer_node`, use_sim_time: true
TODO: subscribes /detected_target (vision_msgs/Detection2DArray)
TODO: tf2 lookup of thermal_link in odom at each detection's timestamp
TODO: publishes  /survivor/position (geometry_msgs/PointStamped, frame odom),
                 /survivor/markers (visualization_msgs/MarkerArray)
TODO: parameters hfov_rad (0.96), width (32), height (24), min_baseline_m (0.5),
      min_ray_angle_deg (10), ground_truth_file

TODO: math
  fx = fy = (width/2) / tan(hfov/2), cx = width/2, cy = height/2
  Ray in thermal_link: d = normalize([1, -(u-cx)/fx, -(v-cy)/fy])
  Transform the origin c and d into odom.
  With two or more rays: A = sum(I - d d^T), b = sum((I - d d^T) c), p = A^-1 b,
    uncertainty = RMS ray-to-point distance. Reject near-parallel rays
    (min_baseline_m, min_ray_angle_deg).
  One-ray fallback: intersect with the floor plane, low confidence.
TODO: markers: estimate, uncertainty sphere, ground truth (from ground_truth_file), error text.
"""


def main():
    # TODO: implement
    pass
