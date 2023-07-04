import rospy
import sys
import cv2
import numpy as np
import message_filters
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from pathlib import Path


class ImgUndistorter:

    def __init__(self, img_topic2sub, cam_info2sub,
                 img2publish, cam_info2publish,
                 scaling_param=1.0,
                 path2calib_file=''):

        self.img_sub = message_filters.Subscriber(img_topic2sub, Image)
        self.cam_info_sub = message_filters.Subscriber(cam_info2sub, CameraInfo)

        # about the policies: http://wiki.ros.org/message_filters#Time_Synchronizer
        self.ts = message_filters.TimeSynchronizer([self.img_sub, self.cam_info_sub], 10)  # exact timespams in headers
        self.ts.registerCallback(self.callback)

        self.img_pub = rospy.Publisher(img2publish, Image, queue_size=1)
        self.cam_info_pub = rospy.Publisher(cam_info2publish, CameraInfo, queue_size=1)

        path2file = Path(path2calib_file)
        if path2file.is_file():
            cam_info_msg = ImgUndistorter.read_calibration_configuration(path2calib_file)
            if cam_info_msg is not None:
                self.K4dist_numpy = np.asarray(cam_info_msg.K).reshape((3, 3)) # camera intrinsic matrix
                self.D4dist = np.asarray(cam_info_msg.D).reshape((1,5))  # distortion coefficients
                self.img_width = cam_info_msg.width
                self.img_height = cam_info_msg.height
                self.undist_cam_info_loaded = True
                rospy.loginfo("K and D for undistorted images loaded from a yaml file.")
            else:
                rospy.logerr("Could not load into cam params")
                return
        else:
            self.undist_cam_info_loaded = False
            self.K4dist_numpy = None # camera intrinsic matrix
            self.D4dist = None  # distortion coefficients
            self.img_width = None
            self.img_height = None

        self.new_cam_matrix_calculated = False
        self.scaling_param = scaling_param
        self.cv_bridge = CvBridge()
        self.K4undist_numpy = None

        self.new_cam_info_msg = CameraInfo()
        self.roi = None

        rospy.loginfo("Image undistorter initialised.")


    def callback(self, ros_dist_img, ros_dist_cam_info):

        # load K and D only once
        if not self.undist_cam_info_loaded:
            self.K4dist_numpy = np.asarray(ros_dist_cam_info.K).reshape((3, 3))
            self.D4dist = np.asarray(ros_dist_cam_info.D).reshape((1,5))
            self.img_width = ros_dist_cam_info.width
            self.img_height = ros_dist_cam_info.height
            if self.K4dist_numpy is not None and self.D4dist is not None:
                rospy.loginfo("K and D for undistorted images loaded from ROS camera msgs.")
            self.undist_cam_info_loaded = True

        # new K and ROI for undistorted images
        if self.undist_cam_info_loaded and not self.new_cam_matrix_calculated:
            shape = (self.img_width, self.img_height)
            self.K4undist_numpy, self.roi = cv2.getOptimalNewCameraMatrix(self.K4dist_numpy,
                                                                          self.D4dist,
                                                                          shape, self.scaling_param, shape)
            self.new_cam_info_msg.K = tuple(self.K4undist_numpy.reshape((9, 1)))

            if self.new_cam_info_msg.K is not None and self.roi is not None:
                rospy.loginfo("K and ROI for undistorted imgs computed.")
                self.new_cam_matrix_calculated = True
            else:
                rospy.logerr("K and ROI for undistorted imgs NOT computed.")

        cv_rgb_dist_image = self.cv_bridge.imgmsg_to_cv2(ros_dist_img, "bgr8")

        # undistort
        cv_undist_img = cv2.undistort(cv_rgb_dist_image, self.K4dist_numpy,
                                      self.D4dist, None, self.K4undist_numpy)

        # crop the image
        x, y, w, h = self.roi
        cv_undist_img = cv_undist_img[y:y + h, x:x + w]

        # convert to ROS image
        ros_undist_img_msg = self.cv_bridge.cv2_to_imgmsg(cv_undist_img, "bgr8")

        # include timestamps and everything else
        ros_undist_img_msg.header.stamp = ros_dist_img.header.stamp
        ros_undist_img_msg.header.frame_id = ros_dist_img.header.frame_id
        ros_undist_img_msg.width = int(w)
        ros_undist_img_msg.height = int(h)
        self.new_cam_info_msg.header.stamp = ros_dist_cam_info.header.stamp  # should be the same as the img above
        self.new_cam_info_msg.header.frame_id = ros_dist_cam_info.header.frame_id
        self.new_cam_info_msg.width = int(w)
        self.new_cam_info_msg.height = int(h)
        # self.new_cam_info_msg.distortion_model = str("plumb_bob")


        # publish
        self.cam_info_pub.publish(self.new_cam_info_msg)
        self.img_pub.publish(ros_undist_img_msg)

    @staticmethod
    def read_calibration_configuration(yaml_fname):

        path2file = Path(yaml_fname)
        if path2file.is_file():
            with open(yaml_fname, "r") as file_handle:
                calib_data = yaml.load(file_handle)
            # Parse
            camera_info_msg = CameraInfo()
            camera_info_msg.width = calib_data["image_width"]
            camera_info_msg.height = calib_data["image_height"]
            camera_info_msg.K = calib_data["camera_matrix"]["data"]
            camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
            camera_info_msg.header.frame_id = calib_data["camera_name"]
            return camera_info_msg
        else:
            return None


def main(args):
    rospy.init_node('pp_lidar2cam_projector', anonymous=False)

    scaling_param = float(rospy.get_param("/pp_lidar2cam_projector/scaling_param"))
    path2calib_file = str(rospy.get_param("/pp_lidar2cam_projector/calib_matrix_from_file"))

    obj = ImgUndistorter(img_topic2sub = "pp/rgb_raw",
                         cam_info2sub="pp/rgb_cam_info",
                         img2publish = "pp/rgb_undist",
                         cam_info2publish = "pp/rgb_undist_info",
                         scaling_param=scaling_param,
                         path2calib_file=path2calib_file)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
