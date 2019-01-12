#include <realsense_transforms_node.h>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <boost/tokenizer.hpp>
#include <tf_conversions/tf_eigen.h>

using namespace tuw;

typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;

RealSenseTransformsNode::RealSenseTransformsNode() : nh_("~")
{
  camera_external_.usable = false;
  nh_.param<std::string>("calib_file", calib_file_, "");
  nh_.param<std::string>("external_calib_file", external_calib_file_, "");
  nh_.param<std::string>("publisher_topic", publisher_topic_, "/r0/realsense");

  sub_transform_ = nh_.subscribe("tf_base_cam_optical_frame",
                                 1000,
                                 &RealSenseTransformsNode::callbackTransform,
                                 this);

  pub_transform_ = nh_.advertise<geometry_msgs::Transform>(publisher_topic_, 10);
}

bool RealSenseTransformsNode::readExternalCalibrationFromFile()
{
  using namespace boost;
  using namespace std;

  if ( external_calib_file_ == std::string(""))
  {
    return false;
  }

  ifstream in(external_calib_file_.c_str());
  if ( !in.is_open())
  {
    std::cout << "Object read: File path is wrong" << std::endl;
  }

  string line;
  vector<vector<string>> parsed_csv;

  while ( getline(in, line))
  {
    if ( line[0] == '#' )
    {
      continue; // allows simple comments
    }
    parsed_csv.push_back(vector<string>());
    Tokenizer tok(line);

//    if (std::distance(tok.begin(),tok.end()) != nr_line_parameters)
//      throw runtime_error("number of entries in csv file wrong. Must provide position (3 variables) plus shape variables (4) for each door.");

    for_each(tok.begin(), tok.end(), [ &parsed_csv ]( string elem )
    {
      parsed_csv.back().push_back(elem);
    });
  }

  assert(parsed_csv.size() == 4);
  assert(parsed_csv[0].size() == 4);
  assert(parsed_csv[1].size() == 4);
  assert(parsed_csv[2].size() == 4);
  assert(parsed_csv[3].size() == 4);

  Eigen::Matrix4d tf_base_cam_optical_frame;
  for ( int r = 0; r < 4; ++r )
  {
    for ( int c = 0; c < 4; ++c )
    {
      try
      {

        tf_base_cam_optical_frame(r, c) = boost::lexical_cast<double>(parsed_csv[r][c]);

      } catch ( const boost::bad_lexical_cast & )
      {
        ROS_INFO("could not cast %s into double, check %s\n", parsed_csv[r][c].c_str(), external_calib_file_.c_str());
      }
    }
  }

  std::cout << "tf_base_cam_optical_frame " << tf_base_cam_optical_frame << std::endl;

  doTransform(tf_base_cam_optical_frame);

  return true;
}

void RealSenseTransformsNode::readInternalCalibrationFromXML()
{
  Eigen::Vector3d trans = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix3d rot;
  rot.setIdentity();

  TiXmlDocument xml;
  if ( xml.LoadFile(calib_file_))
  {
    auto config = xml.FirstChild("Config");
    if ( config )
    {
      TiXmlElement *param = config->FirstChild("param")->ToElement();
      TiXmlElement *sibl = param;
      while ( sibl )
      {
        if ( std::string(sibl->Attribute("name")) == std::string("TranslationLeftRGB"))
        {
          auto value = sibl->FirstChild("value");
          if ( value )
          {
            try
            {
              trans.x() = boost::lexical_cast<double>(value->ToElement()->GetText());
              value = value->NextSibling();
              trans.y() = boost::lexical_cast<double>(value->ToElement()->GetText());
              value = value->NextSibling();
              trans.z() = boost::lexical_cast<double>(value->ToElement()->GetText());
            } catch ( boost::bad_lexical_cast const & )
            {
              ROS_ERROR("lexical cast: cannot cast double (calib xml TranslationLeftRGB)");
            }
          }
        } else if ( std::string(sibl->Attribute("name")) == std::string("RotationLeftRGB"))
        {
          auto vals = sibl->FirstChild("value");

          if ( vals )
          {
            try
            {
              for ( int r = 0; r < rot.rows(); ++r )
              {
                for ( int c = 0; c < rot.cols(); ++c )
                {
                  rot(r, c) = boost::lexical_cast<double>(vals->ToElement()->GetText());
                  vals = vals->NextSibling();
                }
              }
            } catch ( std::runtime_error const & )
            {
              ROS_ERROR("parsing rotation: too few <value></value> arguments provided");
            }
            catch ( boost::bad_lexical_cast const & )
            {
              ROS_ERROR("lexical cast: cannot cast double (calib xml RotationLeftRGB)");
            }
          }
        }

        auto toElementCastable = sibl->NextSibling("param");
        if ( toElementCastable )
        {
          sibl = toElementCastable->ToElement();
        } else
        {
          sibl = nullptr;
        }

      } //end while

      std::stringstream sstr("");
      sstr << "Parsed file for camera parameters\n";
      sstr << "translation: \n";
      sstr << trans << "\n";
      sstr << "rotation: \n";
      sstr << rot << "\n";
      ROS_INFO("%s\n", sstr.str().c_str());

      //converting stuff to meters from cm;
      trans = trans / 100.0;
      camera_internal_.t_leftrgb_origin_.setIdentity();
      camera_internal_.t_leftrgb_origin_.topLeftCorner<3, 3>() = rot;
      camera_internal_.t_leftrgb_origin_.topRightCorner<3, 1>() = trans;
      std::cout << "tf_leftrgb_origin " << std::endl;
      std::cout << camera_internal_.t_leftrgb_origin_ << std::endl;

      //invert stuff without relying on less accurate Eigen functions
      Eigen::Matrix3d rot_t = rot.transpose();
      Eigen::Vector3d trans_t = -trans;
      camera_internal_.t_origin_leftrgb_.setIdentity();
      camera_internal_.t_origin_leftrgb_.topLeftCorner<3, 3>() = rot_t;
      camera_internal_.t_origin_leftrgb_.topRightCorner<3, 1>() = trans_t;
      std::cout << "tf_origin_leftrgb" << std::endl;
      std::cout << camera_internal_.t_origin_leftrgb_ << std::endl;
    }
  } else
  {
    ROS_ERROR("Failed to load config file");
  }
}

void RealSenseTransformsNode::callbackTransform( const geometry_msgs::TransformConstPtr &tf_base_cam_optical_center )
{
  auto &q = tf_base_cam_optical_center->rotation;
  auto &t = tf_base_cam_optical_center->translation;
  Eigen::Quaterniond qe = Eigen::Quaterniond(q.w, q.y, q.z, q.x);
  Eigen::Vector3d te = Eigen::Vector3d(t.x, t.y, t.z);

  Eigen::Matrix4d tf_base_cam_optical_frame;
  tf_base_cam_optical_frame.setIdentity();
  tf_base_cam_optical_frame.topLeftCorner<3, 3>() = qe.toRotationMatrix();
  tf_base_cam_optical_frame.topRightCorner<3, 1>() = te;

  doTransform(tf_base_cam_optical_frame);
}

void RealSenseTransformsNode::doTransform( const Eigen::Matrix4d &tf_base_cam_optical_frame )
{
  camera_external_.t_base_cam_origin_ = tf_base_cam_optical_frame * camera_internal_.t_leftrgb_origin_;
  std::cout << camera_external_.t_base_cam_origin_ << std::endl;
}

void RealSenseTransformsNode::publish()
{
  geometry_msgs::TransformStamped tf_;
  tf_.header.stamp = ros::Time::now();
  tf_.header.frame_id = "r0/base_link";
  tf_.child_frame_id = "r0/mount_camera_tof";

  auto m_rot = camera_external_.t_base_cam_origin_.topLeftCorner<3, 3>();
  Eigen::Vector3d v_trans = camera_external_.t_base_cam_origin_.topRightCorner<3, 1>();

  auto quat = Eigen::Quaterniond(m_rot);
  tf_.transform.rotation.x = quat.x();
  tf_.transform.rotation.y = quat.y();
  tf_.transform.rotation.z = quat.z();
  tf_.transform.rotation.w = quat.w();

  tf_.transform.translation.x = v_trans.x();
  tf_.transform.translation.y = v_trans.y();
  tf_.transform.translation.z = v_trans.z();

  tf_broadcaster_.sendTransform(tf_);
}

int main( int argc, char **argv )
{
  ros::init(argc, argv, "realsense_transforms_node");
  RealSenseTransformsNode node;
  node.readInternalCalibrationFromXML();
  node.readExternalCalibrationFromFile();

  ros::Rate rate(5);
  while ( ros::ok())
  {

    ros::spinOnce();
    rate.sleep();

    node.publish();

  }
  return 0;
}