#include <realsense_transforms_node.h>
#include <Eigen/Geometry>

using namespace tuw;

RealSenseTransformsNode::RealSenseTransformsNode() : nh_( "" )
{
  sub_transform_ = nh_.subscribe( "tf_base_to_image", 1000, &RealSenseTransformsNode::callbackTransform, this );
}

void RealSenseTransformsNode::readXML( const std::string &filename )
{
  TiXmlDocument xml;
  xml.LoadFile( filename );
}

void RealSenseTransformsNode::callbackTransform( const geometry_msgs::TransformConstPtr &tf )
{
  auto &q = tf->rotation;
  auto &t = tf->translation;
  Eigen::Quaterniond qe = Eigen::Quaterniond( q.w, q.y, q.z, q.x );
  Eigen::Vector3d te = Eigen::Vector3d( t.x, t.y, t.z );
  
  Eigen::Matrix4d tf_e;
  tf_e.topLeftCorner<3, 3>() = qe.toRotationMatrix();
  tf_e.topRightCorner<3, 1>() = te;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "door_2d_detector_node" );
  ros::Rate rate( 20 );
  
  while ( ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}