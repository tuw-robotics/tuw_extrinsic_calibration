#include <realsense_transforms_node.h>
#include <Eigen/Geometry>

using namespace tuw;

RealSenseTransformsNode::RealSenseTransformsNode() : nh_( "~" )
{
  nh_.param<std::string>( "calib_file", calib_file_, "" );
  std::cout << calib_file_ << std::endl;
  sub_transform_ = nh_.subscribe( "tf_base_to_image", 1000, &RealSenseTransformsNode::callbackTransform, this );
}

void RealSenseTransformsNode::readXML()
{
  Eigen::Vector4d trans = Eigen::Vector4d( 0, 0, 0, 1 );
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
            } catch (boost::bad_lexical_cast const &)
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
      sstr << "Parsed file \n" << std::endl;
      sstr << "translation: \n" << std::endl;
      sstr << trans << "\n";
      sstr << "rotation: \n";
      sstr << rot << "\n";
      ROS_INFO( "%s", sstr.str().c_str());
    }
  } else
  {
    ROS_ERROR( "Failed to load config file" );
  }
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