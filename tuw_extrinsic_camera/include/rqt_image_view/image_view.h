/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rqt_image_view__ImageView_H
#define rqt_image_view__ImageView_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_view.h>

#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/macros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>

#include <vector>

#include <image_geometry/pinhole_camera_model.h>
#include <boost/circular_buffer.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tuw_measurement_utils/measurements.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tuw_geometry/figure.h>
#include <mutex>

namespace rqt_image_view {
  
  class ImageView
      : public rqt_gui_cpp::Plugin {
  
  Q_OBJECT
  
  public:
    
    ImageView();
    
    virtual void initPlugin( qt_gui_cpp::PluginContext &context );
    
    virtual void shutdownPlugin();
    
    virtual void saveSettings( qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings ) const;
    
    virtual void
    restoreSettings( const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings );
  
  protected slots:
    
    virtual void updateTopicList();
  
  protected:
    
    // deprecated function for backward compatibility only, use getTopics() instead
    ROS_DEPRECATED virtual QList<QString>
    getTopicList( const QSet<QString> &message_types, const QList<QString> &transports );
    
    virtual QSet<QString> getTopics( const QSet<QString> &message_types, const QSet<QString> &message_sub_types,
                                     const QList<QString> &transports );
    
    virtual void selectTopic( const QString &topic );
  
  protected slots:
    
    virtual void onTopicChanged( int index );
    
    virtual void onLeftSliderValChanged( int val );
    
    virtual void onRightSliderValChanged( int val );
    
    virtual void onMousePublish( bool checked );
    
    virtual void onMouseLeft( int x, int y );
    
    virtual void onPubTopicChanged();
    
    virtual void onLaserScanBoxToggle( bool val );
    
    virtual void onFreezeLaserBoxToggle( bool val );
    
    virtual void onFreezeImageBoxToggle( bool val );
    
    virtual void onPublisherButton();
    
    virtual void onRefinePressed();
    
    virtual void onLeftDistanceSliderValChanged( int val );
    
    virtual void onRightDistanceSliderValChanged( int val );
  
  protected:
    
    virtual void drawImages();
    
    virtual void callbackImage( const sensor_msgs::Image::ConstPtr &msg );
    
    virtual void callbackCameraInfo( const sensor_msgs::CameraInfo::ConstPtr &_msg );
    
    virtual void callbackLaser( const sensor_msgs::LaserScan &_laser );
    
    virtual bool updateLaser2Image();
    
    virtual void updateLaser2Map();
    
    virtual bool getStaticTF( const std::string &world_frame, const std::string &source_frame,
                              tf::StampedTransform &_pose, bool debug );
    
    struct PnPData {
    public:
      PnPData() {
      }
      
      std::vector<cv::Point2f> image_points;
      std::vector<cv::Point3f> object_points;
      cv::Matx33d K;
      cv::Mat D, T_CL;
    };
    
    Ui::ImageViewWidget ui_;
    
    QWidget *widget_;
    
    image_transport::Subscriber subscriber_;
    ros::Subscriber sub_camera_info_;
    ros::Subscriber sub_laser_;
    
    boost::circular_buffer<cv::Point2d> clickedPoints_;
    
    cv::Mat conversion_mat_;
    
    std::unique_ptr<tuw::LaserMeasurement> measurement_laser_;
    std::unique_ptr<tuw::ImageMeasurement> measurement_image_;
    std::vector<std::pair<cv::Point2d, cv::Scalar>> laser2image_points_colored_;
    
    std::map<std::string, std::shared_ptr<tf::StampedTransform>> tfMap_;
    std::shared_ptr<image_geometry::PinholeCameraModel> camera_model_;
    
    tuw::FigurePtr figure_local_;
    
    double leftSplitAngle_;
    double rightSplitAngle_;
  
  private:
    
    std::unique_ptr<PnPData> pnp_data_;
    QString arg_topic_name;
    
    bool use_laser_scan_range_;
    bool freeze_laser_scan_;
    bool freeze_image_;
    bool pub_topic_custom_;
    
    int num_gridlines_;
    double restrict_left_laser_max_;
    double restrict_right_laser_max_;
    
    tf::TransformListener listenerTF_;
    
    std::mutex mutex_laser_;
    std::mutex mutex_image_;
  };
  
}

#endif // rqt_image_view__ImageView_H
