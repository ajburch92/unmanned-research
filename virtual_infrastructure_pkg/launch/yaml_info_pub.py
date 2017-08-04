#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

def yaml_to_CameraInfo():
    rospy.init_node("camera_info_publisher", anonymous=True)
    info_publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    # yaml_publisher = rospy.Publisher("yaml_filename", String, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    yaml_fname = rospy.get_param('~camera_yaml')
    # yaml_fname = rospy.get_param('camera_yaml')

    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]

    while not rospy.is_shutdown():
        # yaml_publisher.publish(yaml_fname)
        info_publisher.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        yaml_to_CameraInfo()
    except rospy.ROSInterruptException:
        pass

# rosrun hast yaml_info_pub.py _camera_yaml:=/home/benjamin/.ros/camera_info/16369047.yaml
