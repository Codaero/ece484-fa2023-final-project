#!/bin/bash
rosbag record /zed2/zed_node/depth/camera_info \
    /zed2/zed_node/depth/depth_registered \
    /zed2/zed_node/imu/data \
    /zed2/zed_node/left/camera_info \
    /zed2/zed_node/left/image_rect_color \
    /zed2/zed_node/left/image_rect_color/compressed \
    /zed2/zed_node/odom \
    /zed2/zed_node/rgb/camera_info \
    /zed2/zed_node/rgb/image_rect_color \
    /zed2/zed_node/right/camera_info \
    /zed2/zed_node/right/image_rect_color \
    /zed2/zed_node/right/image_rect_color/compressed \
    /zed2/zed_node/stereo/image_rect_color \
    /zed2/zed_node/stereo/image_rect_color/compressed \
    /novatel/inspva \
    /novatel/imu \
    /clock \
    /tf \