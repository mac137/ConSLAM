#include "velodyne_pointcloud/pointXYZIRT.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class VeloTimeStampCorrector
{
    std::string node_name;
    std::string lidar_topic2sub;
    std::string lidar_topic2publish;
    float half_view_width;

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;

public:
    explicit VeloTimeStampCorrector(ros::NodeHandle* nodeHandle): nh_(*nodeHandle)
    {
        readLaunchParams();
        initializeSubscribers();
        initializePublishers();
    }

void readLaunchParams()
{
    node_name = ros::this_node::getName();
    nh_.param(node_name + "/lidar_topic2sub", lidar_topic2sub, lidar_topic2sub);
    nh_.param(node_name + "/lidar_topic2publish", lidar_topic2publish, lidar_topic2publish);
    nh_.param(node_name + "/half_view_width", half_view_width, half_view_width);
}


//member helper function to set up subscribers;
void initializeSubscribers()
{
    sub_ = nh_.subscribe(lidar_topic2sub, 1, &VeloTimeStampCorrector::callback, this);
}

//member helper function to set up publishers;
void initializePublishers()
{
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(lidar_topic2publish, 1, true);
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &ros_lidar_msg)
{
    float neg_width = -1*half_view_width;
    float time_diff = 0;
    int counter = 0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*ros_lidar_msg, "x"),
                                                 iter_y(*ros_lidar_msg, "y"),
                                                 iter_z(*ros_lidar_msg, "z"),
                                                 iter_time(*ros_lidar_msg, "time");
    while (iter_x != iter_x.end()) {
        if (neg_width < *iter_y && *iter_y < half_view_width) {
                time_diff += *iter_time;
            counter += 1;
        }
        ++iter_x; ++iter_y; ++iter_z, ++iter_time;
    }

    // average time diff
    time_diff = time_diff / float(counter);
    ros::Duration time_diff_dur;
    time_diff_dur.fromSec(time_diff);

    pcl::PointCloud<pcl::VelodynePointXYZIRT>::Ptr point_cloud_pcl (new pcl::PointCloud<pcl::VelodynePointXYZIRT>);
    pcl::fromROSMsg(*ros_lidar_msg, *point_cloud_pcl);

    if (time_diff < 0.) {
        BOOST_FOREACH(pcl::VelodynePointXYZIRT &pcl_velo_point, point_cloud_pcl->points) {
                        pcl_velo_point.time = pcl_velo_point.time + time_diff;
                    }
    }
    else {
        BOOST_FOREACH(pcl::VelodynePointXYZIRT &pcl_velo_point, point_cloud_pcl->points) {
                        pcl_velo_point.time = pcl_velo_point.time - time_diff;
                    }
    }

    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*point_cloud_pcl, tempCloud);
    if (time_diff < 0.) {
        tempCloud.header.stamp = ros_lidar_msg->header.stamp - time_diff_dur;
    }
    else {
        tempCloud.header.stamp = ros_lidar_msg->header.stamp + time_diff_dur;
    }
    pub_.publish(tempCloud);
}

}; // end of VeloTimeStampCorrector


int main (int argc, char** argv)
{
    ros::init (argc, argv, "pp_velo_timestamp_corrector");
    ros::NodeHandle nh;

    VeloTimeStampCorrector bb(&nh);

    ros::spin();

    return EXIT_SUCCESS;
}