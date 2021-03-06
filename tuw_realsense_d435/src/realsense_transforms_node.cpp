#include <realsense_transforms_node.h>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <boost/tokenizer.hpp>
#include <tf_conversions/tf_eigen.h>

using namespace tuw;

typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

RealSenseTransformsNode::ParametersNode::ParametersNode() : nh_( "~" )
{
  bool init_success = nh_.param<std::string>( "calib_file", calib_file_, "" );
  init_success &= nh_.param<std::string>( "external_calib_file", external_calib_file_, "" );
  init_success &= nh_.param<std::string>( "publisher_topic", publisher_topic_, "/r0/realsense" );
  nh_.param<bool>( "debug", debug_, false );
  nh_.param<bool>( "passthrough", passthrough_, false );
  
  if ( !init_success )
  {
    ROS_ERROR( "get parameters failed for at least one param, check launch file" );
  }
}

RealSenseTransformsNode::RealSenseTransformsNode() : nh_(), param_()
{
  camera_external_.usable = false;
  sub_transform_ = nh_.subscribe( "tf_base_cam_optical_frame",
                                  1000,
                                  &RealSenseTransformsNode::callbackTransform,
                                  this );
  pub_transform_ = nh_.advertise<geometry_msgs::Transform>( params().publisher_topic_, 10 );
}

bool RealSenseTransformsNode::readExternalCalibrationFromFile()
{
  using namespace boost;
  using namespace std;
  
  if ( params().external_calib_file_ == std::string( "" ))
  {
    ROS_INFO( "no calib file provided, looking for messages" );
    return false;
  }
  
  ifstream in( params().external_calib_file_.c_str());
  if ( !in.is_open())
  {
    ROS_ERROR( "wrong filepath for external calib file" );
    return false;
  }
  
  string line;
  vector<vector<string>> parsed_csv;
  
  while ( getline( in, line ))
  {
    if ( line[0] == '#' )
    {
      continue; // allows simple comments
    }
    parsed_csv.push_back( vector<string>());
    Tokenizer tok( line );

//    if (std::distance(tok.begin(),tok.end()) != nr_line_parameters)
//      throw runtime_error("number of entries in csv file wrong. Must provide position (3 variables) plus shape variables (4) for each door.");
    
    for_each( tok.begin(), tok.end(), [&parsed_csv]( string elem )
    {
      parsed_csv.back().push_back( elem );
    } );
  }
  
  assert( parsed_csv.size() == 4 );
  assert( parsed_csv[0].size() == 4 );
  assert( parsed_csv[1].size() == 4 );
  assert( parsed_csv[2].size() == 4 );
  assert( parsed_csv[3].size() == 4 );
  
  Eigen::Matrix4d tf_base_cam_optical_frame;
  for ( int r = 0; r < 4; ++r )
  {
    for ( int c = 0; c < 4; ++c )
    {
      try
      {
        tf_base_cam_optical_frame( r, c ) = boost::lexical_cast<double>( parsed_csv[r][c] );
      } catch (const boost::bad_lexical_cast &)
      {
        ROS_INFO( "could not cast %s into double, check %s\n",
                  parsed_csv[r][c].c_str(),
                  params().external_calib_file_.c_str());
      }
    }
  }
  
  std::cout << "tf_base_cam_optical_frame " << tf_base_cam_optical_frame << std::endl;
  doTransform( tf_base_cam_optical_frame );
  
  return true;
}

void RealSenseTransformsNode::readFromXml( const std::string &lens_name, Eigen::Vector3d &trans, Eigen::Matrix3d &rot )
{
  TiXmlDocument xml;
  
  if ( xml.LoadFile( params().calib_file_ ))
  {
    auto config = xml.FirstChild( "Config" );
    if ( config )
    {
      TiXmlElement *param = config->FirstChild( "param" )->ToElement();
      TiXmlElement *sibl = param;
      while ( sibl )
      {
        if ( std::string( sibl->Attribute( "name" )) == std::string( "Translation" ) + lens_name )
        {
          auto value = sibl->FirstChild( "value" );
          if ( value )
          {
            try
            {
              //y is actually switched with x (they do not tell you that of course...)
              trans.y() = boost::lexical_cast<double>( value->ToElement()->GetText());
              value = value->NextSibling();
              trans.x() = boost::lexical_cast<double>( value->ToElement()->GetText());
              value = value->NextSibling();
              trans.z() = boost::lexical_cast<double>( value->ToElement()->GetText());
            } catch (boost::bad_lexical_cast const &)
            {
              ROS_ERROR( "lexical cast: cannot cast double (calib xml TranslationLeftRGB)" );
            }
          }
        } else if ( std::string( sibl->Attribute( "name" )) == std::string( "Rotation" ) + lens_name )
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
    } else
    {
      ROS_ERROR( "Failed to load config file" );
    }
  } else
  {
    ROS_ERROR( "Failed to load config file" );
  }
}

void RealSenseTransformsNode::readInternalCalibrationFromXML()
{
  Eigen::Vector3d trans = Eigen::Vector3d( 0, 0, 0 );
  Eigen::Matrix3d rot;
  rot.setIdentity();
  
  readFromXml( "LeftRGB", trans, rot );
  
  //converting stuff to meters from millimeters;
  trans = trans / 1000.0;
  camera_internal_.t_origin_leftrgb_base_.setIdentity();
  camera_internal_.t_origin_leftrgb_base_.topLeftCorner<3, 3>() = rot;
  camera_internal_.t_origin_leftrgb_base_.topRightCorner<3, 1>() = trans;
  
  //invert stuff without relying on less accurate Eigen functions
  Eigen::Matrix3d rot_t = rot.transpose();
  Eigen::Vector3d trans_t = -trans;
  camera_internal_.t_leftrgb_base_origin_.setIdentity();
  camera_internal_.t_leftrgb_base_origin_.topLeftCorner<3, 3>() = rot_t;
  camera_internal_.t_leftrgb_base_origin_.topRightCorner<3, 1>() = trans_t;
  
  camera_internal_.t_base_opticenter_.setIdentity();
  camera_internal_.t_base_opticenter_.topLeftCorner<3, 3>() = camera_internal_.q_basecam_cam_.toRotationMatrix();
  
  camera_internal_.t_opticenter_base_.setIdentity();
  camera_internal_.t_opticenter_base_.topLeftCorner<3, 3>() = camera_internal_.q_basecam_cam_.toRotationMatrix().transpose();
  
  Eigen::Matrix3d rot_left_right;
  Eigen::Vector3d trans_left_right;
  readFromXml( "LeftRight", trans_left_right, rot_left_right );
  
  trans_left_right = trans_left_right / 1000.0;
  auto trans_right_left = -trans_left_right;
  auto rot_right_left = rot_left_right.transpose();
  
  Eigen::Matrix4d t_right_left;
  t_right_left.setIdentity();
  t_right_left.topLeftCorner<3, 3>() = rot_right_left;
  t_right_left.topRightCorner<3, 1>() = trans_right_left;
  
  camera_internal_.t_infrabase_origin_ = t_right_left * camera_internal_.t_leftrgb_base_origin_;
}

void RealSenseTransformsNode::callbackTransform( const geometry_msgs::TransformConstPtr &tf_base_cam_optical_center )
{
  auto &q = tf_base_cam_optical_center->rotation;
  auto &t = tf_base_cam_optical_center->translation;
  Eigen::Quaterniond qe = Eigen::Quaterniond( q.w, q.y, q.z, q.x );
  Eigen::Vector3d te = Eigen::Vector3d( t.x, t.y, t.z );
  
  Eigen::Matrix4d tf_base_cam_optical_frame;
  tf_base_cam_optical_frame.setIdentity();
  tf_base_cam_optical_frame.topLeftCorner<3, 3>() = qe.toRotationMatrix();
  tf_base_cam_optical_frame.topRightCorner<3, 1>() = te;
  
  doTransform( tf_base_cam_optical_frame );
}

void RealSenseTransformsNode::doTransform( const Eigen::Matrix4d &tf_base_cam_optical_frame )
{
  if ( !param_.passthrough_ )
  {
    camera_external_.t_base_cam_origin_ =
        tf_base_cam_optical_frame *
        camera_internal_.t_opticenter_base_ *
        camera_internal_.t_leftrgb_base_origin_;
    
  } else
  {
    camera_external_.t_base_cam_origin_ = tf_base_cam_optical_frame;
  }
  //@TODO: debug delete
  //camera_external_.t_base_cam_origin_ = tf_base_cam_optical_frame;
  //camera_external_.t_base_cam_origin_.topLeftCorner<3, 3>().setIdentity();
  //std::cout << camera_external_.t_base_cam_origin_ << std::endl;
}

geometry_msgs::TransformStamped RealSenseTransformsNode::make_tf( const Eigen::Matrix4d &tf )
{
  geometry_msgs::TransformStamped tf_stamped;
  
  Eigen::Matrix3d m_rot = tf.topLeftCorner<3, 3>();
  Eigen::Vector3d v_trans = tf.topRightCorner<3, 1>();
  
  auto quat = Eigen::Quaterniond( m_rot );
  tf_stamped.transform.rotation.x = quat.x();
  tf_stamped.transform.rotation.y = quat.y();
  tf_stamped.transform.rotation.z = quat.z();
  tf_stamped.transform.rotation.w = quat.w();
  
  tf_stamped.transform.translation.x = v_trans.x();
  tf_stamped.transform.translation.y = v_trans.y();
  tf_stamped.transform.translation.z = v_trans.z();
  
  return tf_stamped;
}

void RealSenseTransformsNode::publish()
{
  //base to depth (origin)
  geometry_msgs::TransformStamped tf_ = make_tf( camera_external_.t_base_cam_origin_ );
  tf_.header.stamp = ros::Time::now();
  tf_.header.frame_id = "r0/base_link";
  tf_.child_frame_id = params().publisher_topic_;
  tf_broadcaster_.sendTransform( tf_ );
  
  std::cout << "debug: " << (params().debug_ ? "true" : "false") << std::endl;
  
  if ( params().debug_ )
  {
    
    //base to depth (origin)
    Eigen::Matrix4d tf_base_color_eigen =
        camera_external_.t_base_cam_origin_ * camera_internal_.t_origin_leftrgb_base_;
    
    geometry_msgs::TransformStamped tf_base_color = make_tf( tf_base_color_eigen );
    tf_base_color.header.stamp = ros::Time::now();
    tf_base_color.header.frame_id = "r0/base_link";
    tf_base_color.child_frame_id = "r0/realsense_rgb_dbg";
    tf_broadcaster_.sendTransform( tf_base_color );
    
    geometry_msgs::TransformStamped tf_base_optical_rgb = make_tf( camera_external_.t_base_cam_origin_
                                                                   * camera_internal_.t_origin_leftrgb_base_ *
                                                                   camera_internal_.t_base_opticenter_ );
    
    tf_base_optical_rgb.header.stamp = ros::Time::now();
    tf_base_optical_rgb.header.frame_id = "r0/base_link";
    tf_base_optical_rgb.child_frame_id = "r0/realsense_rgb_optical_frame_dbg";
    tf_broadcaster_.sendTransform( tf_base_optical_rgb );
    
    geometry_msgs::TransformStamped tf_base_infra = make_tf(
        camera_external_.t_base_cam_origin_ * camera_internal_.t_infrabase_origin_.inverse());
    
    tf_base_infra.header.stamp = ros::Time::now();
    tf_base_infra.header.frame_id = "r0/base_link";
    tf_base_infra.child_frame_id = "r0/realsense_rgb_infra_frame_dbg";
    tf_broadcaster_.sendTransform( tf_base_infra );
    
  }
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "realsense_transforms_node" );
  RealSenseTransformsNode node;
  
  ros::Rate rate( 5 );
  while ( ros::ok())
  {
    
    node.readInternalCalibrationFromXML();
    node.readExternalCalibrationFromFile();
    node.publish();
    
    ros::spinOnce();
    rate.sleep();
    
  }
  return 0;
}