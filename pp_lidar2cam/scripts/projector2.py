from builtins import RuntimeError

import rospy
import sys
import cv2
import tf
import os
import rosbag
import struct
import numpy as np
import yaml
yaml.warnings({'YAMLLoadWarning': False}) # this to to diosable annoying yaml warning
import message_filters
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from scipy.spatial.transform import Rotation as R
from datetime import datetime
from skimage import io #, exposure, img_as_uint, img_as_float
from pathlib import Path


def lidar_points2numpy(lidar_points):
    numpy_array = np.array([[0], [9999], [9999], [9999], [9999]])
    # print(lidar_points)
    for point in pc2.read_points(lidar_points, field_names=("x", "y", "z", "intensity"), skip_nans=True):
        pt_x = point[0]
        pt_y = point[1]
        pt_z = point[2]
        intensity = [point[3]]
        point_numpy = np.array([[pt_x], [pt_y], [pt_z], [1.0], intensity])

        # distance = np.sqrt(np.sum(np.square(np.array([[pt_x], [pt_y], [pt_z]]))))
        # if distance > 1.0 and pt_x > 0.4:
        numpy_array = np.append(numpy_array, point_numpy, axis=1)

    # this deletes the [0], [9999], [9999] point I artificially inserted to give a shape to the array!
    numpy_array = np.delete(numpy_array, 0, 1)
    #print("=> obtained pointcloud of number of points: {}".format(numpy_array.size))

    return numpy_array


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
        #camera_info_msg.distortion_model = calib_data["camera_model"]
        camera_info_msg.header.frame_id = calib_data["camera_name"]
        return camera_info_msg
    else:
        text = "Lidar2Cam Projector: path to the cam calib file is not valid"
        rospy.logerr(text)
        raise RuntimeError(text)


def intensity2rgb(minimum, maximum, value):
    rgb = [0, 0, 0]
    ratio = 2 * (value-minimum) / (maximum - minimum)
    rgb[2] = int(max([0.0, 255*(1 - ratio)])) #blue
    rgb[0] = int(max([0.0, 255 * (ratio - 1)])) #red
    rgb[1] = 255 - rgb[2] - rgb[0]; #green
    # return (rgb[2], rgb[1], rgb[0])
    return (rgb[0], rgb[2], rgb[1])


def apply_manual_correction_to_the_projection_mtx(projection_mtx):
    assert type(projection_mtx) == np.ndarray, "projection matrix must be of a numpy array"
    assert projection_mtx.shape == (3, 3), "shape of the projeciton matrix is not 3x3"

    # apply my manual correction on cu. This correction on x is equal to the roi form that cv2's function
    # so i thing these guys implemented this a bit incorrectly becuase the calibration looks perfect with this correction
    # i implemented only on cu cos no correction needed in cv - and also the case when cv is very visible is quite rare
    correction_on_cu = 3.0
    return np.array([[projection_mtx[0, 0], projection_mtx[0, 1], projection_mtx[0, 2] + correction_on_cu],
                     [projection_mtx[1, 0], projection_mtx[1, 1], projection_mtx[1, 2]],
                     [projection_mtx[2, 0], projection_mtx[2, 1], projection_mtx[2, 2]]])


class LidarProjector:

    def __init__(self,
                 img_topic2sub: str,
                 lidar_topic2sub: str,
                 control_topic: str,
                 sync_difference_time: float,
                 depth_factor: float,
                 path2cam_calib_yaml: str,
                 input_imgs_are_distorted=True,
                 save_images_and_lidar_bags=True,
                 save_depth_maps=False,
                 create_control=False,
                 that_cv_param_4_cropping_black_dents = 1.0,
                 radius=1,
                 thickness = 2,
                 lidar_point_color_min= 0,
                 lidar_point_color_max = 150,
                 publish_colourised_pcd_msgs=True,
                 ):

        self.input_imgs_are_distorted = input_imgs_are_distorted
        if not self.input_imgs_are_distorted:
            my_text = "INPUT IMAGES MUST BE DISTORTED!"
            rospy.logerr(my_text)
            raise RuntimeError(my_text)

        self.publish_colourised_pcd_msgs = publish_colourised_pcd_msgs
        # the path must be here to apply manual correection to this matrix
        self.path2cam_calib_yaml = path2cam_calib_yaml

        self.save_images_and_lidar_bags = save_images_and_lidar_bags
        self.save_depth_maps = save_depth_maps
        self.create_control = create_control
        self.depth_factor = depth_factor

        self.cam_info = read_calibration_configuration(self.path2cam_calib_yaml)
        self.init_cam_mtx = np.reshape(self.cam_info.K, newshape=(3, 3))
        self.dist_coeffcients = np.reshape(self.cam_info.D, newshape=(1, 5))
        self.projection4undist = np.zeros(shape=(3, 3)) #np.reshape(self.cam_info.P, newshape=(3, 3)) # this will be replaced in the code later
        self.previous_msg_timestamp = None
        self.roi = None
        self.mtx4undist = None

        # print(self.cam_info.K)
        # print(np.reshape(self.cam_info.K, newshape=(3,3)))
        # print(self.cam_info.D)

        # not controling much
        self.that_cv_param_4_cropping_black_dents = that_cv_param_4_cropping_black_dents
        self.lidar_point_radius_on_control_img = radius
        self.lidar_point_thickenss_on_control_img = thickness
        self.lidar_point_color_min_on_control_img = lidar_point_color_min
        self.lidar_point_color_max_on_control_img = lidar_point_color_max

        if self.save_images_and_lidar_bags is not None and self.save_images_and_lidar_bags:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            self.dir_path = os.path.join(dir_path, "..", "tmp")
            self.dir_path = os.path.join(self.dir_path, datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
            if not os.path.exists(self.dir_path):
                self.dir_rgb = os.path.join(self.dir_path, "rgb")
                os.makedirs(self.dir_rgb)
                self.dir_lidar_points = os.path.join(self.dir_path, "lidar_points")
                os.makedirs(self.dir_lidar_points)
                if self.save_depth_maps:
                    self.dir_d = os.path.join(self.dir_path, "lidar_d")
                    os.makedirs(self.dir_d)
                    if self.create_control:
                        self.dir_control = os.path.join(self.dir_path, "control")
                        os.makedirs(self.dir_control)
        self.counter = 0

        self.img_sub = message_filters.Subscriber(img_topic2sub, Image)
        #self.cam_info_sub = message_filters.Subscriber(cam_info2sub, CameraInfo)
        self.lidar_sub = message_filters.Subscriber(lidar_topic2sub, PointCloud2)

        # about the policies: http://wiki.ros.org/message_filters#Time_Synchronizer
        # self.ts = message_filters.TimeSynchronizer([self.img_sub, self.lidar_sub], 10)  # exact timespams in headers

        # 210519 for rgb 20 fps: this was established as the best previously 0.022
        # 210520 for rgb 45 fps: 0.01
        self.time_diff_between_msgs = sync_difference_time  # not exact matching + required headers!
        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.lidar_sub], 10,
                                                              self.time_diff_between_msgs,
                                                              allow_headerless=False,
                                                              reset=True)
        self.ts.registerCallback(self.callback)
        # self.initialise_synchroniser()
        if self.create_control:
            self.img_pub = rospy.Publisher(control_topic, Image, queue_size=1)

        if self.publish_colourised_pcd_msgs:
            self.lidar_rgb_pub = rospy.Publisher(lidar_topic2sub+"_rgb", PointCloud2, queue_size=1)

        self.cv_bridge = CvBridge()

        if 'nir' in img_topic2sub:
            tf_camera = '/pp_nir'
        else:
            tf_camera = '/pp_rgb'
        self.lidar2cam_translation20, self.lidar2cam_rotation20 = LidarProjector.get_lidar2cam_transform(tf.TransformListener(), tf_camera)
        self.computation_extrinsic_matrix20 = LidarProjector.get_computation_extrinsic_mtx(self.lidar2cam_rotation20, self.lidar2cam_translation20)
        print(self.computation_extrinsic_matrix20)



        rospy.loginfo("Lidar projector initialised.")

    def get_init_cam_mtx(self):
        return self.init_cam_mtx

    def get_init_dist_coeffs(self):
        return self.dist_coeffcients
        #k1 = -0.061116
        #k2 = 0.130805
        #p1 = -0.000317
        #p2 = -0.000202
        #k3 = 0.000000
        #return np.array([k1, k2, p1, p2, k3])

    def callback(self, ros_dist_img, lidar_points):

        # ros img to cv img
        cv_image = self.cv_bridge.imgmsg_to_cv2(ros_dist_img, "bgr8")

        if self.input_imgs_are_distorted:
            if self.mtx4undist is None:
                height, width, channels = cv_image.shape
                shape = (width, height)
                self.mtx4undist, self.roi = cv2.getOptimalNewCameraMatrix(self.get_init_cam_mtx(),
                                                                          self.get_init_dist_coeffs(),
                                                                          shape,
                                                                          self.that_cv_param_4_cropping_black_dents,
                                                                          shape)
                # after cropping the image (in cv_image = cv_image[y:y + h, x:x + w]) I also need to adjust the mtx4undist
                np.copyto(self.projection4undist, self.mtx4undist)
                x, y, w, h = self.roi
                self.projection4undist[0, 2] = self.projection4undist[0, 2] - x
                self.projection4undist[1, 2] = self.projection4undist[1, 2] - y
                # i noticed that that applying this small change below makes the projection looks better for both NIR and RGB cams
                self.projection4undist = apply_manual_correction_to_the_projection_mtx(self.projection4undist)

                # # print the projection mtx for the console
                text = "Projection matrix is: {}".format(self.projection4undist)
                print(text)
                rospy.loginfo(text)

            cv_image = cv2.undistort(cv_image, self.get_init_cam_mtx(),
                                     self.get_init_dist_coeffs(), None, self.mtx4undist)
            x, y, w, h = self.roi
            # print("x is: {}".format(x))
            # print("y is: {}".format(y))

            cv_image = cv_image[y:y + h, x:x + w]

        height, width, channels = cv_image.shape

        # save the image
        if self.save_images_and_lidar_bags:
            dir_img = os.path.join(self.dir_rgb, str(str(self.counter)+".png"))
            cv2.imwrite(dir_img, cv_image)

            depth_map = np.zeros(shape=(height, width), dtype=np.uint16)

            dir_lidar_points = os.path.join(self.dir_lidar_points, str(str(self.counter)+".bag"))
            bag = rosbag.Bag(dir_lidar_points, 'w')
            try:
                bag.write(topic='velodyne_points', msg=lidar_points, t=lidar_points.header.stamp)
            finally:
                bag.close()

        if self.publish_colourised_pcd_msgs:
            lidar_colourized_points = list()

        if self.save_depth_maps or self.create_control or self.publish_colourised_pcd_msgs:
            points_numpy = lidar_points2numpy(lidar_points)

            points3D_in_cam = np.matmul(self.computation_extrinsic_matrix20, points_numpy[0:4, :])
            projected_points_homogenous = np.matmul(self.projection4undist, points3D_in_cam)
            uv_rows = projected_points_homogenous[0:2, :].reshape((2, points_numpy.shape[1]))
            w_row = projected_points_homogenous[2, :].reshape((1, points_numpy.shape[1]))
            continous_projected_points = uv_rows / w_row
            # dont discretize here cos you will use int() later for that
            # discretised_projected_points = np.floor(continous_projected_points)

            for proj_point, point_in_3D, point_numpy in zip(continous_projected_points.transpose(),
                                               points3D_in_cam.transpose(),
                                               points_numpy.transpose()):
                # check if the point is within the width and the hight of the map
                if point_numpy[0] > 1.1 and 0 < proj_point[0] < width and 0 < proj_point[1] < height:

                    # int() does the floor operation
                    discretized_projected_point = (int(proj_point[0]), int(proj_point[1]))

                    if self.publish_colourised_pcd_msgs:
                        (r, g, b) = cv_image[discretized_projected_point[1], discretized_projected_point[0]]
                        a = 255
                        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                        colourized_point = [point_numpy[0], point_numpy[1], point_numpy[2], point_numpy[4], rgb]
                        lidar_colourized_points.append(colourized_point)


                    #decide on the colour
                    colour = intensity2rgb(self.lidar_point_color_min_on_control_img,
                                           self.lidar_point_color_max_on_control_img,
                                           point_numpy[4])

                    cv_image = cv2.circle(cv_image, discretized_projected_point,
                                          self.lidar_point_radius_on_control_img,
                                          colour,
                                          self.lidar_point_thickenss_on_control_img)

                    if self.save_depth_maps:
                        # check if the pixel is already assigned or assigned with higher value (the smaller value should be taken)
                        current_depth_value = depth_map[discretized_projected_point[1], discretized_projected_point[0]]
                        candidate_depth_value = self.depth_factor * np.sqrt(np.sum(np.square(point_in_3D)))
                        if current_depth_value == 0:
                            depth_map[discretized_projected_point[1], discretized_projected_point[0]] = candidate_depth_value
                        elif candidate_depth_value < current_depth_value:
                            depth_map[discretized_projected_point[1], discretized_projected_point[0]] = candidate_depth_value

                elif self.publish_colourised_pcd_msgs:
                    # (r, g, b) = (None, None, None) # None's dont work! Error occurs
                    # a = None
                    (r, g, b) = (255, 255, 255)
                    a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    colourized_point = [point_numpy[0], point_numpy[1], point_numpy[2], point_numpy[4], rgb]
                    lidar_colourized_points.append(colourized_point)



            # save the lidar depth map
            if self.save_depth_maps:
                dir_depth = os.path.join(self.dir_d, str(str(self.counter)+".png"))
                io.imsave(dir_depth, depth_map)
                # also save the control img cos storing the depth map alone does not make sense
                dir_img = os.path.join(self.dir_control, str(str(self.counter) + ".png"))
                cv2.imwrite(dir_img, cv_image)

            if self.create_control:
                # publish the projected lidar points onto the rgb image into ROS
                ros_img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
                ros_img_msg.header.stamp = ros_dist_img.header.stamp
                ros_img_msg.header.frame_id = ros_dist_img.header.frame_id
                self.img_pub.publish(ros_img_msg)

            # publish colourized pointcloud
            if self.publish_colourised_pcd_msgs:
                fields = [PointField('x', 0, PointField.FLOAT32, 1),
                          PointField('y', 4, PointField.FLOAT32, 1),
                          PointField('z', 8, PointField.FLOAT32, 1),
                          PointField('intensity', 12, PointField.FLOAT32, 1),
                          PointField('rgba', 16, PointField.UINT32, 1),
                          ]
                header = lidar_points.header
                ros_point_cloud_msg = point_cloud2.create_cloud(header, fields, lidar_colourized_points)
                self.lidar_rgb_pub.publish(ros_point_cloud_msg)

        if self.save_images_and_lidar_bags or self.save_depth_maps:
            text = "Saved data point number {}".format(self.counter)
            rospy.loginfo(text)

        self.counter += 1

    @staticmethod
    def get_computation_extrinsic_mtx(rot, trans):

        # this quaternion agrees with the one from get_lidar2cam_transform
        # q = tf.transformations.quaternion_from_euler(rot[2], rot[1], rot[0])
        r = R.from_quat(rot)

        # according to https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_rotvec.html
        # R.from_rotvec() takes the arguments in xyz order as referenced to
        # https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation#Rotation_vector
        # r = R.from_rotvec(rot)

        # q_sci = r.as_quat()
        # print(q_sci)
        # rot_mat = r.as_matrix()
        rot_mat = r.as_dcm()
        # rot_mat = np.array([[0.0, 0.0, 1.0],
        #              [-1.0, 0.0, 0.0],
        #              [0.0, -1.0, 0.0]])
        # print(rot_mat)
        my_transformation_mtx = np.append(rot_mat, [[trans[0]], [trans[1]], [trans[2]]], 1)
        # my_transformation_mtx = np.append(rot_mat, [[0], [0], [0]], 1)
        my_transformation_mtx = np.append(my_transformation_mtx, [[0., 0., 0., 1.]], 0)
        my_computation_mtx = np.linalg.inv(my_transformation_mtx)
        my_computation_mtx = np.delete(my_computation_mtx, 3, 0)
        return my_computation_mtx

    @staticmethod
    def get_lidar2cam_transform(tf_listener, camera_tf):
        trans = None
        counter_local = 0
        counter_global = 0
        while trans is None:
            try:
                # /velodyne first makes this agree with the tranform input from the calibration matrix
                # (trans, rot) = tf_listener.lookupTransform('/velodyne', '/rotated_pp_rgb', rospy.Time(0))
                # (trans, rot) = self.tf_listener.lookupTransform('/rotated_pp_rgb', '/velodyne', rospy.Time(0))
                # rotation is in quaternion! and let it be like this. If you want to convert, use r = R.from_quat(rot)
                # and then rot_mat = r.as_matrix() and so on https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html
                (trans, rot) = tf_listener.lookupTransform('/velodyne', camera_tf, rospy.Time(0))

                # rot = tf.transformations.euler_from_quaternion(rot)
                # this maes sure that the elements or the rot are in the xyz order
                # this is probably [yaw, pitch, roll]
                # rot = list((rot[2], rot[1], rot[0]))
                # print(trans)
                # print(rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # below is just a boiler plate code to provide for the situation when TF between two frames cannot be found
                if counter_local == 5000:
                    text = "Looking for TF between /velodyne and {} ...".format(camera_tf)
                    rospy.logwarn(text)
                    counter_local = 0
                counter_local = counter_local + 1

                counter_global = counter_global + 1
                if counter_global == 500000:
                    text = "TF between /velodyne and {} not found.".format(camera_tf)
                    rospy.logerr(text)
                    raise RuntimeError(text)

        rospy.loginfo("TF between /velodyne and {} found".format(camera_tf))
        return trans, rot


def main(args):
    rospy.init_node('Velo2UndisortCamProjector', anonymous=True)

    path2cam_calib_yaml = str(rospy.get_param("/projector2/path2cam_calib_yaml"))
    sync_difference_time = float(rospy.get_param("/projector2/sync_difference_time"))
    img_topic2sub = str(rospy.get_param("/projector2/subscriber_img_topic"))
    lidar_topic2sub = str(rospy.get_param("/projector2/subscriber_lidar_topic"))
    control_topic = str(rospy.get_param("/projector2/publisher_img_topic"))
    save_depth_maps = rospy.get_param("/projector2/save_depth_maps")
    save_images_and_lidar_bags = rospy.get_param("/projector2/save_images_and_lidar_bags")
    create_control = rospy.get_param("/projector2/create_control")
    input_images_are_distorted = bool(rospy.get_param("/projector2/input_images_are_distorted"))
    depth_factor = float(rospy.get_param("/projector2/depth_factor"))
    publish_colourised_pcd_msgs = bool(rospy.get_param("/projector2/publish_colourised_pcd_msgs"))

    obj = LidarProjector(
        path2cam_calib_yaml=path2cam_calib_yaml,
        img_topic2sub=img_topic2sub,
        lidar_topic2sub=lidar_topic2sub,
        control_topic=control_topic,
        sync_difference_time=sync_difference_time,
        depth_factor=depth_factor,
        input_imgs_are_distorted=input_images_are_distorted,
        save_images_and_lidar_bags=save_images_and_lidar_bags,
        save_depth_maps=save_depth_maps,
        create_control=create_control,
        publish_colourised_pcd_msgs=publish_colourised_pcd_msgs,
    )

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
