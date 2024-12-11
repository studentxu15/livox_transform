#pragma once
#ifndef HEADER_H
#define HEADER_H

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <livox_ros_driver/CustomMsg.h>


struct PointXYZIRT {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 ///< laser intensity reading
  uint16_t ring;                   ///< laser ring number
  float time;                      ///< laser time reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)  //
                                  (float, y, y)                             //
                                  (float, z, z)                             //
                                  (float, intensity, intensity)             //
                                  (uint16_t, ring, ring)                    //
                                  (float, time, time)                       //
)

typedef PointXYZIRT RTPoint;
typedef pcl::PointCloud<RTPoint> RTPointCloud;

class ParamServer
{
public:
	ros::NodeHandle nh;

    std::string PROJECT_NAME;
    std::vector<double> livox_TR_;
    std::string livox_raw_message_name_;
    std::string livox_points_message_name_;
    std::string livox_frame_;

    ParamServer()
    {
        ros::NodeHandle private_nh("~");
        nh.param<std::string>("PROJECT_NAME", PROJECT_NAME, "livox_transform");
        private_nh.param<std::vector<double>>("livox_TR", livox_TR_, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        private_nh.param<std::string>("livox_raw_message_name", livox_raw_message_name_, "/livox/lidar");
        private_nh.param<std::string>("livox_points_message_name", livox_points_message_name_, "/livox_points");
        private_nh.param<std::string>("livox_frame", livox_frame_, "livox_frame");
    }
};
#endif // HEADER_H