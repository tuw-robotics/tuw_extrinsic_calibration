#include <tuw_extrinsic_camera/laser2cam_calibration_node.h>

int main( int argc, char **argv ) {
  ros::init( argc, argv, "door_2d_detector_node" );
  
  Laser2CamCalibrationNode node;
  
  while ( ros::ok()) {
    ros::spinOnce();
    
  }
  return 0;
}