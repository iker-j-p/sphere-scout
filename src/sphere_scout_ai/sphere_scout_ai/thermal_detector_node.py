"""thermal_detector_node: finds warm blobs in the thermal image.

Owner: TODO(kickoff)

TODO: rclpy node `thermal_detector_node`, use_sim_time: true
TODO: subscribes /thermal/image_raw (sensor_msgs/Image, 16-bit, kelvin x 100)
TODO: publishes  /detected_target (vision_msgs/Detection2DArray)
TODO: parameters t_low_c, t_high_c, min_pixels (defaults: TODO(kickoff)), resolution (0.01)
TODO: convert temp_c = pixels * resolution - 273.15; debug-log the frame's min and max degC
TODO: threshold t_low_c <= temp_c <= t_high_c, connected components, drop blobs smaller than
      min_pixels, publish one Detection2D per blob with the image's header stamp.
"""


def main():
    # TODO: implement
    pass
