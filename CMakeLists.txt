cmake_minimum_required(VERSION 3.10.2)
project(vpr_dataset_creator_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  std_srvs
  tf
)
