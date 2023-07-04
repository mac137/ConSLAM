//
// Created by Maciej Trzeciak on 15/02/2022.
//

#ifndef VELODYNE_POINTCLOUD_POINTXYZIRT_H
#define VELODYNE_POINTCLOUD_POINTXYZIRT_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace pcl {
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                           (uint16_t, ring, ring) (float, time, time)
)

#endif //VELODYNE_POINTCLOUD_POINTXYZIRT_H
