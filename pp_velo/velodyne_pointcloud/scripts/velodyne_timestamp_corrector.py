import rospy
import sys
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


def read_cam_lidar_COMPUTATION_extrinsics():
    # 210726 these are the pointpix extrinsics taken from projector2 after good intr and extre calibs
    return np.array([[0.0302375783, -0.999537248,  0.00331354288, -0.00677560000],
                     [0.000891025887, -0.00328810277, -0.999994197, -0.0656465000],
                     [0.999542343,  0.0302403553,  0.000791189298, -0.0288792000]])


def lidar_points2numpy(lidar_points):
    numpy_array = np.array([[0], [9999], [9999], [9999], [9999]], dtype=np.float16)
    counter = 0
    for point in pc2.read_points(lidar_points, field_names=("x", "y", "z", "time"), skip_nans=True):
        if point[0] > 1.1:
            # take only every 20 point (anyway we cal only average so it is very reasonable)
            if counter == 20:
                numpy_array = np.append(numpy_array, np.array([[point[0]], [point[1]], [point[2]], [1.0], [point[3]]], dtype=np.float16), axis=1)
                counter = 0
            counter = counter + 1

    # this deletes the [0], [9999], [9999] point I artificially inserted to give a shape to the array!
    numpy_array = np.delete(numpy_array, 0, 1)
    #print("=> obtained pointcloud of number of points: {}".format(numpy_array.size))
    return numpy_array

    # return [np.array([[point[0]], [point[1]], [point[2]], [1.0], [point[3]]]) for point in pc2.read_points(lidar_points, field_names=("x", "y", "z", "time"), skip_nans=True) if point[0] > 1.2]


class LidarProjector:

    def __init__(self,
                 lidar_topic2sub: str,
                 lidar_topic2publish: str,
                 debug: False,
                 ):

        self.sub = rospy.Subscriber(lidar_topic2sub, PointCloud2, self.callback)
        self.pub = rospy.Publisher(lidar_topic2publish, PointCloud2, queue_size=10)
        self.debug = debug

        self.counter = 1


    def callback(self, velo_orig_msg):

        # start1 = rospy.Time.now()

        average_time_diff_from_numpy = 0.0

        # these points are of numpy array and their x-coordinate > 1.1 already!
        points = lidar_points2numpy(velo_orig_msg)

        number_of_points = points.shape[1]
        if number_of_points < 20:
            test = "Time stamp correction in lidar msgs not calculated cos less than {} lidar point passed through the conditions".format(number_of_points)
            rospy.logwarn(test)
            return
        else:
            points3D_in_cam = np.matmul(read_cam_lidar_COMPUTATION_extrinsics(), points[0:4, :])

            # dim 0 is X - which is the same as in lidar Y
            mask_on_y = np.abs(points3D_in_cam[0, :]) < 8.0
            # masked_points = mask_on_y * points
            # masked_points = np.ma.masked_equal(masked_points, 0)
            # test_average = np.average(masked_points[4,:])

            time_diffs = points[4, :][mask_on_y]
            #print(time_diffs)
            average_time_diff_from_numpy = float(np.average(time_diffs))



        # end1 = rospy.Time.now()
        # dur1 = end1 - start1

        # test_bad = rospy.Duration(-1 * average_time_diff_from_numpy)
        # test_good = rospy.Duration().from_sec(-1 * average_time_diff_from_numpy)

        if average_time_diff_from_numpy < 0:
            average_time_diff_ros_duration = rospy.Duration().from_sec(-1*average_time_diff_from_numpy)
            velo_orig_msg.header.stamp = velo_orig_msg.header.stamp - average_time_diff_ros_duration
        else:
            average_time_diff_ros_duration = rospy.Duration().from_sec(average_time_diff_from_numpy)
            velo_orig_msg.header.stamp = velo_orig_msg.header.stamp + average_time_diff_ros_duration

        # start2 = rospy.Time.now()
        # time_diffs = list()
        #
        # for point in pc2.read_points(velo_orig_msg, field_names=("x", "y", "z", "time"), skip_nans=True):
        #     # a very small lidar rotation against the cam is neglibeable here so i dont apply it here
        #     if point[0] > 1.2 and abs(point[1]) < 15.0: # z is always within the cam view
        #         time_diffs.append(point[3])
        #
        # average_time_diff = 0.0
        # if len(time_diffs) != 0:
        #     average_time_diff = float(np.average(np.array(time_diffs)))
        # else:
        #     test = "Time stamp correction in lidar msgs not calculated cos no lidar point went through the conditions"
        #     rospy.logwarn(test)

        # end2 = rospy.Time.now()
        # dur2 = end2 - start2
        # print("numpy dur:    {}".format(dur1.to_sec()))
        # print("ordinary dur: {}".format(dur2.to_sec()))
        # print("numpy average diff:       {}".format(average_time_diff_from_numpy))
        # print("ordinary average diff:    {}".format(average_time_diff))
        # print("")


        # self.counter = self.counter + 1
        #
        # if self.debug:
        #     print("message {}".format(self.counter))
        #
        #     number_of_points = len(time_diffs)
        #
        #     text1 = "number of points within: {}".format(number_of_points)
        #     rospy.logdebug(text1)
        #     print(text1)
        #
        #     text2 = "average time difference from the header timestamp: {}".format(average_time_diff)
        #     rospy.logdebug(text2)
        #     print(text2)
        #
        # if average_time_diff < 0:
        #     average_time_diff_ros_duration = rospy.Duration(-1*average_time_diff)
        #     velo_orig_msg.header.stamp = velo_orig_msg.header.stamp - average_time_diff_ros_duration
        # else:
        #     average_time_diff_ros_duration = rospy.Duration(average_time_diff)
        #     velo_orig_msg.header.stamp = velo_orig_msg.header.stamp + average_time_diff_ros_duration

        self.pub.publish(velo_orig_msg)


def main(args):
    rospy.init_node('Velodyne_timestamp_corrector_node', anonymous=False)

    lidar_topic2sub = str(rospy.get_param("/velodyne_timestamp_corrector/lidar_topic2sub"))
    lidar_topic2publish = str(rospy.get_param("/velodyne_timestamp_corrector/lidar_topic2publish"))
    debug = bool(rospy.get_param("/velodyne_timestamp_corrector/debug"))

    # lidar_topic2sub = "velodyne_points"
    # lidar_topic2publish = "pp/velodyne_points_4_sync"
    # debug = False

    obj = LidarProjector(
        lidar_topic2sub=lidar_topic2sub,
        lidar_topic2publish=lidar_topic2publish,
        debug=debug,
    )

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
