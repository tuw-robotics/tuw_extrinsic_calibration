#include <realsense_transforms_node.h>
#include <Eigen/Geometry>

using namespace tuw;

RealSenseTransformsNode::RealSenseTransformsNode() : nh_( "~" )
{
  nh_.param<std::string>( "calib_file", calib_file_, "" );
  sub_transform_ = nh_.subscribe( "tf_base_cam_optical_frame",
                                  1000,
                                  &RealSenseTransformsNode::callbackTransform,
                                  this );
}

void RealSenseTransformsNode::readXML()
{
  Eigen::Vector3d trans = Eigen::Vector3d( 0, 0, 0 );
  Eigen::Matrix3d rot;
  rot.setIdentity();
  
  TiXmlDocument xml;
  if ( xml.LoadFile( calib_file_ ))
  {
    auto config = xml.FirstChild( "Config" );
    if ( config )
    {
      TiXmlElement *param = config->FirstChild( "param" )->ToElement();
      TiXmlElement *sibl = param;
      while ( sibl )
      {
        if ( std::string( sibl->Attribute( "name" )) == std::string( "TranslationLeftRGB" ))
        {
          auto value = sibl->FirstChild( "value" );
          if ( value )
          {
            try
            {
              trans.x() = boost::lexical_cast<double>( value->ToElement()->GetText());
              value = value->NextSibling();
              trans.y() = boost::lexical_cast<double>( value->ToElement()->GetText());
              value = value->NextSibling();
              trans.z() = boost::lexical_cast<double>( value->ToElement()->GetText());
            } catch (boost::bad_lexical_cast const &)
            {
              ROS_ERROR( "lexical cast: cannot cast double (calib xml TranslationLeftRGB)" );
            }
          }
        } else if ( std::string( sibl->Attribute( "name" )) == std::string( "RotationLeftRGB" ))
        {
          auto vals = sibl->FirstChild( "value" );
          
          if ( vals )
          {
            try
            {
              for ( int r = 0; r < rot.rows(); ++r )
              {
                for ( int c = 0; c < rot.cols(); ++c )
                {
                  rot( r, c ) = boost::lexical_cast<double>( vals->ToElement()->GetText());
                  vals = vals->NextSibling();
                }
              }
            } catch (std::runtime_error const &)
            {
              ROS_ERROR( "parsing rotation: too few <value></value> arguments provided" );
            }
            catch (boost::bad_lexical_cast const &)
            {
              ROS_ERROR( "lexical cast: cannot cast double (calib xml RotationLeftRGB)" );
            }
          }
        }
        
        auto toElementCastable = sibl->NextSibling( "param" );
        if ( toElementCastable )
        {
          sibl = toElementCastable->ToElement();
        } else
        {
          sibl = nullptr;
        }
        
      } //end while
      
      std::stringstream sstr( "" );
      sstr << "Parsed file \n";
      sstr << "translation: \n";
      sstr << trans << "\n";
      sstr << "rotation: \n";
      sstr << rot << "\n";
      ROS_INFO( "%s", sstr.str().c_str());
      
      //converting stuff to meters from cm;
      trans = trans / 100.0;
      camera_internal_.t_leftrgb_origin_.setIdentity();
      camera_internal_.t_leftrgb_origin_.topLeftCorner<3, 3>() = rot;
      camera_internal_.t_leftrgb_origin_.topRightCorner<3, 1>() = trans;
      
      //invert stuff without relying on less accurate Eigen functions
      rot = rot.transpose();
      trans = -trans;
      camera_internal_.t_leftrgb_origin_.setIdentity();
      camera_internal_.t_origin_leftrgb_.topLeftCorner<3, 3>() = rot;
      camera_internal_.t_origin_leftrgb_.topRightCorner<3, 1>() = trans;
    }
  } else
  {
    ROS_ERROR( "Failed to load config file" );
  }
}

void RealSenseTransformsNode::callbackTransform( const geometry_msgs::TransformConstPtr &tf_base_cam_optical_center )
{
  auto &q = tf_base_cam_optical_center->rotation;
  auto &t = tf_base_cam_optical_center->translation;
  Eigen::Quaterniond qe = Eigen::Quaterniond( q.w, q.y, q.z, q.x );
  Eigen::Vector3d te = Eigen::Vector3d( t.x, t.y, t.z );
  
  Eigen::Matrix4d tf_base_cam_optical_frame;
  tf_base_cam_optical_frame.topLeftCorner<3, 3>() = qe.toRotationMatrix();
  tf_base_cam_optical_frame.topRightCorner<3, 1>() = te;
  
  camera_external_.t_base_cam_origin_ = tf_base_cam_optical_frame * camera_internal_.t_leftrgb_origin_
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "realsense_transforms_node" );
  RealSenseTransformsNode node;
  node.readXML();
  
  ros::Rate rate( 20 );
  while ( ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}