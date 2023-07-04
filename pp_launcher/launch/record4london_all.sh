#!/bin/bash

rosbag record pp_points/synced2rgb pp_rgb/synced2points pp_nir/synced2points imu/data pp_rgb/camera_info pp_nir/camera_info
