#!/bin/bash
# SPEAR camera + control bridge launcher.
# Bridges Gazebo Fortress topics to ROS 2 Humble for the live demo.
# Run this in its own terminal — it stays attached.

set -e

source /opt/ros/humble/setup.bash

echo "============================================================"
echo "  SPEAR Bridge — Gazebo Fortress <-> ROS 2 Humble"
echo "============================================================"
echo "  Bridging:"
echo "    /model/interceptor/cmd_vel    (ROS->Gazebo)"
echo "    /model/target/cmd_vel         (ROS->Gazebo)"
echo "    /world/spear_intercept/dynamic_pose/info  (Gazebo->ROS)"
echo "    /interceptor/camera           (Gazebo->ROS, image)"
echo "    /interceptor/camera_info      (Gazebo->ROS, info)"
echo "============================================================"

ros2 run ros_gz_bridge parameter_bridge \
  /model/interceptor/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /model/target/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist \
  /world/spear_intercept/dynamic_pose/info@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V \
  /interceptor/camera@sensor_msgs/msg/Image@ignition.msgs.Image \
  /interceptor/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo
