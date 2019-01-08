#ifndef REALSENSE_TRANSFORMS_NODE
#define REALSENSE_TRANSFORMS_NODE

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <tinyxml.h>

namespace tuw
{
  
  class RealSenseTransformsNode
  {
  
  public:
    
    RealSenseTransformsNode();
    
    void readXML( const std::string &filename );
    
    void callbackTransform( const geometry_msgs::TransformConstPtr &tf );
  
  private:
    ros::Subscriber sub_transform_;
    ros::NodeHandle nh_;
    
  };
  
}

#endif