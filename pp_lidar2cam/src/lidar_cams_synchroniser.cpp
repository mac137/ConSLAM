#include <stdlib.h>
#include <stdio.h>
#include <filesystem>
#include <ctime>
#include <ros/package.h>

#include <iostream>
#include <ros/ros.h>

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"

class LidarCamOdometrySync{
public:
    LidarCamOdometrySync():
            private_nh_("~"){
        //intilaization
        LidarCamOdometrySync::GetParam();
        
        image_pub = nh_.advertise<sensor_msgs::Image>(cam_topic2publish, 1, true);
        pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic2publish, 1, true);

        rgb_sub.subscribe(nh_, rgb_topic, 1);
        pointcloud_sub.subscribe(nh_, pointcloud_topic, 1);

        if (save_nir_too)
        {
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
            message_filters::Synchronizer<SyncPolicy> *sync_policy;

            nir_sub.subscribe(nh_, nir_topic, 1);
            // sync policy according to 7.3 in http://wiki.ros.org/message_filters based on timestamps
            sync_policy = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), pointcloud_sub, rgb_sub, nir_sub);
            sync_policy->setMaxIntervalDuration(ros::Duration(max_diff_time));
            sync_policy->registerCallback(boost::bind(&LidarCamOdometrySync::callback_with_nir, this, _1, _2, _3));
        } else
        {
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicy;
            message_filters::Synchronizer<SyncPolicy> *sync_policy;

            sync_policy = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), pointcloud_sub, rgb_sub);
            sync_policy->setMaxIntervalDuration(ros::Duration(max_diff_time));
            sync_policy->registerCallback(boost::bind(&LidarCamOdometrySync::callback_without_nir, this, _1, _2));
        }

        counter = 0;
    }

    ~LidarCamOdometrySync(){};

    void Run(){
        ROS_INFO_STREAM("Synchroniser initialised with max_time_diff = " << max_diff_time);
        ros::spin();
    }

private:
    void GetParam(){

        node_name = ros::this_node::getName();
        private_nh_.param(node_name + "/sync_difference_time", max_diff_time, max_diff_time);
        private_nh_.param(node_name + "/subscriber_lidar_topic", pointcloud_topic, pointcloud_topic);
        private_nh_.param(node_name + "/subscriber_rgb_topic", rgb_topic, rgb_topic);
        private_nh_.param(node_name + "/lidar_topic2publish", pointcloud_topic2publish, pointcloud_topic2publish);
        private_nh_.param(node_name + "/camera_topic2publish", cam_topic2publish, cam_topic2publish);
        private_nh_.param(node_name + "/save_nir_too", save_nir_too, save_nir_too);
        if (save_nir_too)
        {
            private_nh_.param(node_name + "/subscriber_nir_topic", nir_topic, nir_topic);
        }

    void callback_with_nir(const sensor_msgs::PointCloud2::ConstPtr& point_cloud,
                  const sensor_msgs::Image::ConstPtr& rgb_image,
                  const sensor_msgs::Image::ConstPtr& nir_image)
    {
            auto time_stamp = ros::Time::now();

            rosbag::Bag bag;
            std::string path2bag = path2folder + "/" + std::to_string(counter) + ".bag";
            bag.open(path2bag, rosbag::bagmode::Write);
            bag.write(pointcloud_topic, time_stamp, point_cloud);
            bag.write(rgb_topic, time_stamp, rgb_image);
            bag.write(nir_topic, time_stamp, nir_image);
            bag.close();
            std::string text = "Bag " + std::to_string(counter) + "with points, rgb and nir saved.";
            ROS_INFO_STREAM("Rgb, nir and pointcloud saved in bag number " << counter);

            counter++;
    }

    void callback_without_nir(const sensor_msgs::PointCloud2::ConstPtr& point_cloud,
                           const sensor_msgs::Image::ConstPtr& rgb_image)
    {
    	image_pub.publish(rgb_image);
    	pointcloud_pub.publish(point_cloud);
    }


private:
    ros::NodeHandle nh_, private_nh_;

//    // publishers
    ros::Publisher image_pub;
    ros::Publisher pointcloud_pub;

    // subscribers
    message_filters::Subscriber<sensor_msgs::Image>  rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image>  nir_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub;

    std::string node_name;

    std::string pointcloud_topic;
    std::string rgb_topic;
    std::string nir_topic;

    std::string pointcloud_topic2publish;
    std::string cam_topic2publish;

    float max_diff_time;
    int counter;

    std::string path2rgb;
    bool save_nir_too;
    std::string path2nir;
    std::string path2lidar_points;
    std::string path2folder;

};

int main(int argc, char **argv){
    ROS_INFO("Starting lidar-cams synchroniser");
    ros::init(argc, argv, "sync_lidar_cams_node");

    LidarCamOdometrySync sync_cam_lidar_node;
    sync_cam_lidar_node.Run();

    ros::shutdown();
    return EXIT_SUCCESS;
}
