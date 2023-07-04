import rospy

from sensor_msgs.msg import Image, Imu
from sensor_msgs.msg import PointCloud2, PointField


class MonitorTopics:

    def __init__(self,
                 monitor_lidar, lidar_topic,
                 monitor_imu, imu_topic,
                 monitor_rgb, rgb_topic,
                 monitor_nir, nir_topic
                 ):

        self.every_X_seconds = 2
        self.lidar_topic2sub = lidar_topic
        self.imu_topic2sub = imu_topic
        self.rgb_topic2sub = rgb_topic
        self.nir_topic2sub = nir_topic
        self.monitor_lidar = monitor_lidar
        self.monitor_imu = monitor_imu
        self.monitor_rgb = monitor_rgb
        self.monitor_nir = monitor_nir

        # DONT CHANGE THESE - BASIC LOGIC
        self.lidar_publishes = True
        self.imu_publishes = True
        self.rgb_publishes = True
        self.nir_publishes = True

        rospy.loginfo("The following topics will be monitored:")

        if self.monitor_lidar:
            self.lidar_sub = rospy.Subscriber(self.lidar_topic2sub, PointCloud2, self.lidar_callback)
            self.lidar_publishes = False
            rospy.loginfo(str(self.lidar_topic2sub))

        if self.monitor_imu:
            self.imu_sub = rospy.Subscriber(self.imu_topic2sub, Imu, self.imu_callback)
            self.imu_publishes = False
            rospy.loginfo(str(self.imu_topic2sub))

        if self.monitor_rgb:
            self.rgb_sub = rospy.Subscriber(self.rgb_topic2sub, Image, self.rgb_callback)
            self.rgb_publishes = False
            rospy.loginfo(str(self.rgb_topic2sub))

        if self.monitor_nir:
            self.nir_sub = rospy.Subscriber(self.nir_topic2sub, Image, self.nir_callback)
            self.nir_publishes = False
            rospy.loginfo(str(self.nir_topic2sub))

        self.timer = rospy.Timer(rospy.Duration(self.every_X_seconds), self.timer_callback)
        self.error_msg = "does not publish data ... "
        self.counter = 1

        rospy.loginfo("")
        rospy.loginfo("Monitoring started ...")

    def lidar_callback(self, lidar_msg):
        self.lidar_publishes = True

    def imu_callback(self, imu_msg):
        self.imu_publishes = True

    def rgb_callback(self, rgb_msg):
        self.rgb_publishes = True

    def nir_callback(self, nir_msg):
        self.nir_publishes = True

    def timer_callback(self, timer):
        if not self.lidar_publishes and self.monitor_lidar:
            msg = ">>> LIDAR " + self.error_msg + str(self.counter)
            rospy.logerr(msg)
        if not self.imu_publishes and self.monitor_imu:
            msg = ">>> IMU " + self.error_msg + str(self.counter)
            rospy.logerr(msg)
        if not self.rgb_publishes and self.monitor_rgb:
            msg = ">>> RGB " + self.error_msg + str(self.counter)
            rospy.logerr(msg)
        if not self.nir_publishes and self.monitor_nir:
            msg = ">>> NIR " + self.error_msg + str(self.counter)
            rospy.logerr(msg)

        if self.lidar_publishes and self.imu_publishes and self.rgb_publishes and self.nir_publishes:
            msg = "All topics work ... " + str(self.counter)
            rospy.loginfo(msg)

        if self.monitor_lidar:
            self.lidar_publishes = False
        if self.monitor_imu:
            self.imu_publishes = False
        if self.monitor_rgb:
            self.rgb_publishes = False
        if self.monitor_nir:
            self.nir_publishes = False
        self.counter += 1


def main():
    rospy.init_node('pp_monitor_topics', anonymous=True)

    monitor_lidar = bool(rospy.get_param("/pp_topic_monitor/monitor_lidar"))
    lidar_topic = str(rospy.get_param("/pp_topic_monitor/lidar_topic"))

    monitor_imu = bool(rospy.get_param("/pp_topic_monitor/monitor_imu"))
    imu_topic = str(rospy.get_param("/pp_topic_monitor/imu_topic"))

    monitor_rgb = bool(rospy.get_param("/pp_topic_monitor/monitor_rgb"))
    rgb_topic = str(rospy.get_param("/pp_topic_monitor/rgb_topic"))

    monitor_nir = bool(rospy.get_param("/pp_topic_monitor/monitor_nir"))
    nir_topic = str(rospy.get_param("/pp_topic_monitor/nir_topic"))

    obj = MonitorTopics(monitor_lidar=monitor_lidar, lidar_topic=lidar_topic,
                        monitor_imu=monitor_imu, imu_topic=imu_topic,
                        monitor_rgb=monitor_rgb, rgb_topic=rgb_topic,
                        monitor_nir=monitor_nir, nir_topic=nir_topic)

    rospy.spin()


if __name__ == '__main__':
    main()
