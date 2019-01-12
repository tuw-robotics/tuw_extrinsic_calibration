#ifndef REALSENSE_TRANSFORMS_NODE
#define REALSENSE_TRANSFORMS_NODE

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <tinyxml.h>

namespace tuw
{

  class RealSenseTransformsNode
  {

  public:
    struct CameraInternalExtrinsics
    {
    public:
      Eigen::Matrix4d t_origin_leftrgb_;
      Eigen::Matrix4d t_leftrgb_origin_;
    };

    struct CameraExternalExtrinsics
    {
    public:
      Eigen::Matrix4d t_base_cam_origin_;
      bool usable;
    };

    RealSenseTransformsNode();

    bool readExternalCalibrationFromFile();

    void readInternalCalibrationFromXML();

    void doTransform( const Eigen::Matrix4d &tf_base_cam_optical_frame );

    void publish();

    void callbackTransform( const geometry_msgs::TransformConstPtr &tf_base_cam_origin );

  private:
    ros::Subscriber sub_transform_;
    ros::Publisher pub_transform_;
    std::string publisher_topic_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::NodeHandle nh_;
    std::string calib_file_;
    std::string external_calib_file_;
    CameraInternalExtrinsics camera_internal_;
    CameraExternalExtrinsics camera_external_;

  };

}

#endif