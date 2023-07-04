import numpy as np
import rospy
import sys
import threading
import yaml
import cv2
from signal import signal, SIGINT
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from vimba import *
# this must be like this to properly install Python package in ROS
# see http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile
from pp_alvium_driver.alvium_driver import get_camera, setup_camera
from cv_bridge import CvBridge, CvBridgeError
from pathlib import Path


class MyHandler:
    def __init__(self, img_publisher, cam_info_publisher,
                 yaml_fname, absolute_exposure_time, hardware_delay_correction):
        self.shutdown_event = threading.Event()
        self.img_publisher = img_publisher
        self.cam_info_publisher = cam_info_publisher
        self.half_exposure_time_nanoseconds = rospy.Duration(0, (absolute_exposure_time / 2) * 1000) # *1000 to convert from microseconds (10^-6) to nanoseconds (10^-9)
        self.hardware_delay_correction = rospy.Duration().from_sec(hardware_delay_correction) #0.026 seems ok, 0.025 seems ok too
        #print(self.hardware_correction.to_sec())
        self.bridge = CvBridge()

        if yaml_fname is not None:
            rospy.loginfo("Reading parameters from the calibration file: " + yaml_fname)
            self.cam_info_params_msg = MyHandler.read_calibration_configuration(yaml_fname)
            if self.cam_info_params_msg is None:
                rospy.logerr("Could not read from the calibration file!")

    def __call__(self, cam: Camera, frame: Frame):
        raw_time_stamp = rospy.Time.now()

        if frame.get_status() == FrameStatus.Complete:
            # print('{} acquired {}'.format(cam, frame), flush=True)
            try:
                # encodings rgb8 odwraca kolory, see here: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
                # DONT RESIZE HERE BECUASE IT EATS UP 70% OF M1PRO xD
                # ros_img_msg = self.bridge.cv2_to_imgmsg(MyHandler.resize_img(frame.as_opencv_image(), 2064, 1544), encoding="mono8")
                ros_img_msg = self.bridge.cv2_to_imgmsg(frame.as_opencv_image(), encoding="mono8")

                time_stamp = raw_time_stamp - self.half_exposure_time_nanoseconds - self.hardware_delay_correction
                ros_img_msg.header.stamp = time_stamp
                self.cam_info_params_msg.header.stamp = time_stamp

                ros_img_msg.header.frame_id = self.cam_info_params_msg.header.frame_id
                self.img_publisher.publish(ros_img_msg)
                self.cam_info_publisher.publish(self.cam_info_params_msg)

            except CvBridgeError as e:
                print(e)

        cam.queue_frame(frame)

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
            #camera_info_msg.R = calib_data["rectification_matrix"]["data"]
            #camera_info_msg.P = calib_data["projection_matrix"]["data"]
            camera_info_msg.distortion_model = calib_data["distortion_model"]
            camera_info_msg.header.frame_id = calib_data["camera_name"]
            return camera_info_msg
        else:
            return None

    def handler_f(self, signal_received, frame):
        # Handle any cleanup here
        self.close_properly()
        # self.f.close()
        # print('SIGINT or CTRL-C detected. Exiting gracefully')
        exit(0)

    def close_properly(self):
        # self.f.close()
        self.shutdown_event.set()
        rospy.loginfo("Alvium camera's thread closing ...")
        return

    @staticmethod
    def resize_img(img: np.ndarray, width: int, height: int):
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
        return img[..., np.newaxis]


def main(args):

    try:
        yaml_fname = str(rospy.get_param("/ros_alvium_nir/path2cam_calib_yaml"))
        cam_id = str(rospy.get_param("/ros_alvium_nir/camera_id"))
        frequency = int(rospy.get_param("/ros_alvium_nir/frequency"))
        cam_info = bool(rospy.get_param("/ros_alvium_nir/publish_camera_info"))
        cam_info_topic = str(rospy.get_param("/ros_alvium_nir/published_caminfo_topic"))
        cam_img_topic = str(rospy.get_param("/ros_alvium_nir/published_img_topic"))
        exposure_time = int(rospy.get_param("/ros_alvium_nir/exposure_time"))
        hardware_delay_correction = float(rospy.get_param("/ros_alvium_nir/hardware_delay_correction"))

    except:
        text = "NIR camera driver could not get launch parameters"
        rospy.logerr(text)
        raise RuntimeError(text)


    pub_img = rospy.Publisher(cam_img_topic, Image, queue_size=1)
    pub_cam_info = rospy.Publisher(cam_info_topic, CameraInfo, queue_size=1)
    rospy.init_node('pp_alvium_python_driver', anonymous=True)
    rospy.loginfo("Alvium driver initialised")

    # ##########################################
    # ### NIR SETTINGS - REPLACING PARAMS.YAML
    # ##########################################
    # cam_id = "DEV_1AB22C00C28B"
    # frequency = 60
    # cam_info = True
    # # yaml_fname = "/home/maciej/ros1_wss/pp_collector/src/pp_alvium_driver/calib/210421_NIR.yml"
    # # yaml_fname = "/home/maciej/ros/py3_ws/src/pp_alvium_driver/calib/210421_NIR.yml"
    # yaml_fname = "../calib/210421_NIR.yml"
    # ##########################################

    rospy.loginfo("Opening Vimba ...")
    with Vimba.get_instance():
        with get_camera(cam_id) as cam:

            # Start Streaming, wait for five seconds, stop streaming
            setup_camera(cam, exposure_time)
            # handler = Handler4ros(pub_img, pub_cam_info, yaml_fname, absolute_exposure_time, hardware_delay_correction)
            handler = MyHandler(img_publisher=pub_img, cam_info_publisher=pub_cam_info,
                                yaml_fname=yaml_fname, absolute_exposure_time=exposure_time,
                                hardware_delay_correction=hardware_delay_correction)

            # this handles CTRL+C to close the node properly
            signal(SIGINT, handler.handler_f)
            rospy.loginfo("Alvium camera of id={} opened ...".format(cam_id))

            try:
                # Start Streaming with a custom a buffer of 10 Frames (defaults to 5)
                cam.start_streaming(handler=handler, buffer_count=10)
                handler.shutdown_event.wait()

            finally:
                cam.stop_streaming()
                rospy.loginfo("Alvium camera driver closed properly.")
                # handler.close_properly()
                # rospy.on_shutdown(handler.close_properly())

    # that does not seem to work so uncommented
    # rospy.on_shutdown(close_cam(cam))

if __name__ == '__main__':
    main(sys.argv)
