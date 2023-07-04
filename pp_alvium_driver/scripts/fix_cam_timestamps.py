import rospy
# import sys
# import cv2
# import numpy as np
# import message_filters
# import yaml
# from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo
# from pathlib import Path


absolute_exposure_time = 2200
hardware_delay_correction = 0.026
half_exposure_time_nanoseconds = rospy.Duration(0, (absolute_exposure_time / 2) * 1000) # *1000 to convert from microseconds (10^-6) to nanoseconds (10^-9)
hardware_delay_correction = rospy.Duration().from_sec(hardware_delay_correction)
pub_img = None

def callback(ros_dist_img: Image):
    ros_dist_img.header.stamp = ros_dist_img.header.stamp - half_exposure_time_nanoseconds - hardware_delay_correction
    pub_img.publish(ros_dist_img)
    print("corrected img published")



def main():
    rospy.init_node('correct_cameras_timestamps', anonymous=False)

    # scaling_param = float(rospy.get_param("/pp_lidar2cam_projector/scaling_param"))
    # path2calib_file = str(rospy.get_param("/pp_lidar2cam_projector/calib_matrix_from_file"))
    img_topic2sub = "pp/rgb_raw"
    img_topic2pub = "pp/rgb_raw_time_ok"
    # scaling_param = 1.0

    global pub_img
    pub_img = rospy.Publisher(img_topic2pub, Image, queue_size=5)
    rospy.Subscriber(img_topic2sub, Image, callback)

    rospy.spin()


if __name__ == '__main__':
    main()
