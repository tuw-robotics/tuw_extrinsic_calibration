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
    struct ParametersNode
    {
      ParametersNode();
      
      ros::NodeHandle nh_;
      std::string publisher_topic_;
      std::string external_calib_file_;
      std::string calib_file_;
      bool debug_;
      
    };
    
    struct CameraInternalExtrinsics
    {
    public:
      CameraInternalExtrinsics()
      {
        t_origin_leftrgb_base_.setIdentity();
        t_leftrgb_base_origin_.setIdentity();
        t_opticenter_base_.setIdentity();
        t_base_opticenter_.setIdentity();
        t_infrabase_origin_.setIdentity();
        q_basecam_cam_.x() = -0.5;
        q_basecam_cam_.y() = 0.5;
        q_basecam_cam_.z() = -0.5;
        q_basecam_cam_.w() = 0.5;
      }
      
      Eigen::Matrix4d t_origin_leftrgb_base_;
      Eigen::Matrix4d t_leftrgb_base_origin_;
      Eigen::Matrix4d t_opticenter_base_;
      Eigen::Matrix4d t_base_opticenter_;
      Eigen::Matrix4d t_infrabase_origin_;
      Eigen::Quaterniond q_basecam_cam_;
    };
    
    struct CameraExternalExtrinsics
    {
    public:
      CameraExternalExtrinsics()
      {
        t_base_cam_origin_.setIdentity();
        usable = false;
      }
      
      Eigen::Matrix4d t_base_cam_origin_;
      bool usable;
    };
    
    RealSenseTransformsNode();
    
    bool readExternalCalibrationFromFile();
    
    void readInternalCalibrationFromXML();
    
    void doTransform( const Eigen::Matrix4d &tf_base_cam_optical_frame );
    
    geometry_msgs::TransformStamped make_tf( const Eigen::Matrix4d &tf );
    
    void publish();
    
    void readFromXml( const std::string &lens_name_, Eigen::Vector3d &trans, Eigen::Matrix3d &rot );
    
    void callbackTransform( const geometry_msgs::TransformConstPtr &tf_base_cam_origin );
    
    ParametersNode &params()
    {
      return param_;
    }
  
  private:
    ros::Subscriber sub_transform_;
    ros::Publisher pub_transform_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::NodeHandle nh_;
    CameraInternalExtrinsics camera_internal_;
    CameraExternalExtrinsics camera_external_;
    
    ParametersNode param_;
    
  };
  
}

#endif