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

#include <rqt_image_view/image_view.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <tf/transform_datatypes.h>
#include <marker_msgs/FiducialDetection.h>
#include <opencv2/highgui.hpp>
#include <iterator>

namespace rqt_image_view {
  
  ImageView::ImageView()
      : rqt_gui_cpp::Plugin(), widget_( 0 ), num_gridlines_( 0 ) {
    setObjectName( "ImageView" );
  }
  
  void ImageView::initPlugin( qt_gui_cpp::PluginContext &context ) {
    widget_ = new QWidget();
    ui_.setupUi( widget_ );
    
    if ( context.serialNumber() > 1 ) {
      widget_->setWindowTitle( widget_->windowTitle() + " (" + QString::number( context.serialNumber()) + ")" );
    }
    context.addWidget( widget_ );
    
    updateTopicList();
    ui_.topics_combo_box->setCurrentIndex( ui_.topics_combo_box->findText( "" ));
    connect( ui_.topics_combo_box, SIGNAL( currentIndexChanged( int )), this, SLOT( onTopicChanged( int )) );
    
    ui_.refresh_topics_push_button->setIcon( QIcon::fromTheme( "view-refresh" ));
    connect( ui_.refresh_topics_push_button, SIGNAL( pressed()), this, SLOT( updateTopicList()) );
    
    //ui_.save_as_image_push_button->setIcon( QIcon::fromTheme( "document-save-as" ));
    //connect( ui_.save_as_image_push_button, SIGNAL( pressed()), this, SLOT( saveImage()) );
    
    // set topic name if passed in as argument
    const QStringList &argv = context.argv();
    if ( !argv.empty()) {
      arg_topic_name = argv[0];
      selectTopic( arg_topic_name );
    }
    pub_topic_custom_ = false;
    
    ui_.image_frame->setImage( QImage());
    ui_.laser_frame->setImage( QImage());
    
    QRegExp rx(
        "([a-zA-Z/][a-zA-Z0-9_/]*)?" ); //see http://www.ros.org/wiki/ROS/Concepts#Names.Valid_Names (but also accept an empty field)
    ui_.publish_click_location_topic_line_edit->setValidator( new QRegExpValidator( rx, this ));
    connect( ui_.image_frame, SIGNAL( mouseLeft( int, int )), this, SLOT( onMouseLeft( int, int )) );
    connect( ui_.publish_click_location_topic_line_edit, SIGNAL( editingFinished()), this, SLOT( onPubTopicChanged()) );
    
    connect( ui_.laser_scan_checkbox, SIGNAL( toggled( bool )), this, SLOT( onLaserScanBoxToggle( bool )) );
    connect( ui_.freeze_laser_checkbox, SIGNAL( toggled( bool )), this, SLOT( onFreezeLaserBoxToggle( bool )) );
    connect( ui_.freeze_image_checkbox, SIGNAL( toggled( bool )), this, SLOT( onFreezeImageBoxToggle( bool )) );
    connect( ui_.publisher_button, SIGNAL( pressed()), this, SLOT( onPublisherButton()) );
    connect( ui_.refine_button, SIGNAL( pressed()), this, SLOT( onRefinePressed()) );
    
    ui_.laser_scan_checkbox->toggled( false );
    ui_.freeze_laser_checkbox->toggled( false );
    ui_.freeze_image_checkbox->toggled( false );
    
    ui_.sliderLeftLaser->setMinimum( 0 );
    ui_.sliderRightLaser->setMinimum( 0 );
    
    ui_.sliderLeftLaser->setMaximum( 360 );
    ui_.sliderRightLaser->setMaximum( 360 );
    
    ui_.sliderLeftLaser->setValue( 0 );
    ui_.sliderRightLaser->setValue( 360 );
    
    connect( ui_.sliderLeftLaser, SIGNAL( valueChanged( int )), this, SLOT( onLeftSliderValChanged( int )) );
    connect( ui_.sliderRightLaser, SIGNAL( valueChanged( int )), this, SLOT( onRightSliderValChanged( int )) );
    connect( ui_.sliderLeftLaserD, SIGNAL( valueChanged( int )), this, SLOT( onLeftDistanceSliderValChanged( int )) );
    connect( ui_.sliderRightLaserD, SIGNAL( valueChanged( int )), this, SLOT( onRightDistanceSliderValChanged( int )) );
    
    ui_.sliderRightLaserD->setMinimum( 0 );
    ui_.sliderRightLaserD->setMinimum( 0 );
    
    ui_.sliderLeftLaserD->setMaximum( 100 );
    ui_.sliderRightLaserD->setMaximum( 100 );
    
    ui_.sliderLeftLaserD->setValue( 100 );
    ui_.sliderRightLaserD->setValue( 100 );
    
    leftSplitAngle_ = -M_PI / 2.0;
    rightSplitAngle_ = M_PI / 2.0;
    
    clickedPoints_ = boost::circular_buffer<cv::Point2d>( 4 );
    
    figure_local_.reset( new tuw::Figure( "laser2map" ));
    figure_local_->init( 500, 500, -0.2, 10, -5, 5, M_PI / 2.0, 0.25, 0.7 );
  }
  
  void ImageView::drawImages() {
    std::lock_guard<std::mutex> lock( mutex_image_ );
    
    if ( clickedPoints_.size()) {
      for ( int i = 0; i < clickedPoints_.size(); ++i ) {
        cv::circle( conversion_mat_, clickedPoints_[i], 2, cv::Scalar( 0, 255, 0 ), 2 );
        if ( measurement_image_ && measurement_image_->getCameraModel()) {
          cv::Point3d pt3d = measurement_image_->getCameraModel()->projectPixelTo3dRay( clickedPoints_[i] );
        }
      }
    }
    
    if ( laser2image_points_.size()) {
      for ( size_t i = 0; i < laser2image_points_.size(); ++i ) {
        cv::circle( conversion_mat_, laser2image_points_[i], 2, cv::Scalar( 255, 0, 0 ), 2 );
      }
    }
    
    if ( conversion_mat_.rows > 0 && conversion_mat_.cols > 0 ) {
      QImage image( conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0],
                    QImage::Format_RGB888 );
      ui_.image_frame->setImage( image );
      ui_.image_frame->setInnerFrameFixedSize( image.size());
    }
  }
  
  void ImageView::shutdownPlugin() {
    subscriber_.shutdown();
  }
  
  void ImageView::onFreezeImageBoxToggle( bool val ) {
    freeze_image_ = val;
  }
  
  void ImageView::onFreezeLaserBoxToggle( bool val ) {
    freeze_laser_scan_ = val;
  }
  
  void ImageView::onLaserScanBoxToggle( bool val ) {
    use_laser_scan_range_ = val;
  }
  
  void ImageView::onLeftSliderValChanged( int val ) {
    if ( measurement_laser_ ) {
      double angle_min = 0, angle_max = 1;
      {
        std::lock_guard<std::mutex> lock( mutex_laser_ );
        angle_min = measurement_laser_->getLaser().angle_min;
        angle_max = measurement_laser_->getLaser().angle_max;
      }
      rightSplitAngle_ =
          ((static_cast<double>(val) / static_cast<double>(ui_.sliderLeftLaser->maximum())) * (angle_max - angle_min)) +
          angle_min;
      updateLaser2Map();
      drawImages();
    }
  }
  
  void ImageView::onLeftDistanceSliderValChanged( int val ) {
    restrict_left_laser_max_ = static_cast<double>(val) / static_cast<double>(ui_.sliderLeftLaserD->maximum());
    
    //Parentheses needed, otherwise self inflicted deadlock
    {
      std::lock_guard<std::mutex> lock( mutex_laser_ );
      double range_max = 30;
      if ( measurement_laser_ ) {
        //range_max = measurement_laser_->getLaser().range_max;
      }
      restrict_left_laser_max_ *= range_max;
    }
    
    std::cout << "lm " << restrict_left_laser_max_ << std::endl;
    
    updateLaser2Map();
    drawImages();
  }
  
  void ImageView::onRightSliderValChanged( int val ) {
    if ( measurement_laser_ ) {
      double angle_min = 0, angle_max = 1;
      {
        std::lock_guard<std::mutex> lock( mutex_laser_ );
        angle_min = measurement_laser_->getLaser().angle_min;
        angle_max = measurement_laser_->getLaser().angle_max;
      }
      leftSplitAngle_ =
          ((static_cast<double>(val) / static_cast<double>(ui_.sliderRightLaser->maximum())) *
           (angle_max - angle_min)) + angle_min;
      updateLaser2Map();
      drawImages();
    }
  }
  
  void ImageView::onRightDistanceSliderValChanged( int val ) {
    restrict_right_laser_max_ = static_cast<double>(val) / static_cast<double>(ui_.sliderRightLaserD->maximum());
    
    //Parentheses needed, otherwise self inflicted deadlock
    {
      std::lock_guard<std::mutex> lock( mutex_laser_ );
      double range_max = 30;
      if ( measurement_laser_ ) {
        //range_max = measurement_laser_->getLaser().range_max;
      }
      restrict_right_laser_max_ *= range_max;
    }
    
    std::cout << "rm " << restrict_right_laser_max_ << std::endl;
    
    updateLaser2Map();
    drawImages();
  }
  
  void ImageView::saveSettings( qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings ) const {
    QString topic = ui_.topics_combo_box->currentText();
    qDebug( "ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
    instance_settings.setValue( "topic", topic );
    plugin_settings.setValue( "topic", topic );
    //instance_settings.setValue( "laser_box", ui_.laser_scan_checkbox->isChecked());
  }
  
  void ImageView::restoreSettings( const qt_gui_cpp::Settings &plugin_settings,
                                   const qt_gui_cpp::Settings &instance_settings ) {
    //bool zoom1_checked = instance_settings.value( "zoom1", false ).toBool();
    //ui_.zoom_1_push_button->setChecked( zoom1_checked );
    //bool dynamic_range_checked = instance_settings.value( "dynamic_range", false ).toBool();
    //ui_.dynamic_range_check_box->setChecked( dynamic_range_checked );
    //
    //double max_range = instance_settings.value( "max_range", ui_.max_range_double_spin_box->value()).toDouble();
    //ui_.max_range_double_spin_box->setValue( max_range );
    //
    //num_gridlines_ = instance_settings.value( "num_gridlines", ui_.num_gridlines_spin_box->value()).toInt();
    //ui_.num_gridlines_spin_box->setValue( num_gridlines_ );
    //
    //QString topic = instance_settings.value( "topic", "" ).toString();
    //// don't overwrite topic name passed as command line argument
    //if ( !arg_topic_name.isEmpty()) {
    //  arg_topic_name = "";
    //} else {
    //  //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
    //  selectTopic( topic );
    //}
    //
    //bool publish_click_location = instance_settings.value( "publish_click_location", false ).toBool();
    //ui_.publish_click_location_check_box->setChecked( publish_click_location );
    //
    //QString pub_topic = instance_settings.value( "mouse_pub_topic", "" ).toString();
    //ui_.publish_click_location_topic_line_edit->setText( pub_topic );
    //
    //bool toolbar_hidden = instance_settings.value( "toolbar_hidden", false ).toBool();
    //hide_toolbar_action_->setChecked( toolbar_hidden );
    //
    //bool smooth_image_checked = instance_settings.value( "smooth_image", false ).toBool();
    //ui_.smooth_image_check_box->setChecked( smooth_image_checked );
    //
    //rotate_state_ = static_cast<RotateState>(instance_settings.value( "rotate", 0 ).toInt());
    //if ( rotate_state_ >= ROTATE_STATE_COUNT )
    //  rotate_state_ = ROTATE_0;
    //syncRotateLabel();
  }
  
  void ImageView::updateTopicList() {
    QSet<QString> message_types;
    message_types.insert( "sensor_msgs/Image" );
    QSet<QString> message_sub_types;
    message_sub_types.insert( "sensor_msgs/CompressedImage" );
    
    // get declared transports
    QList<QString> transports;
    image_transport::ImageTransport it( getNodeHandle());
    std::vector<std::string> declared = it.getDeclaredTransports();
    for ( std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++ ) {
      //qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
      QString transport = it->c_str();
      
      // strip prefix from transport name
      QString prefix = "image_transport/";
      if ( transport.startsWith( prefix )) {
        transport = transport.mid( prefix.length());
      }
      transports.append( transport );
    }
    
    QString selected = ui_.topics_combo_box->currentText();
    
    // fill combo box
    QList<QString> topics = getTopics( message_types, message_sub_types, transports ).values();
    topics.append( "" );
    qSort( topics );
    ui_.topics_combo_box->clear();
    for ( QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++ ) {
      QString label( *it );
      label.replace( " ", "/" );
      ui_.topics_combo_box->addItem( label, QVariant( *it ));
    }
    
    // restore previous selection
    selectTopic( selected );
  }
  
  QList<QString> ImageView::getTopicList( const QSet<QString> &message_types, const QList<QString> &transports ) {
    QSet<QString> message_sub_types;
    return getTopics( message_types, message_sub_types, transports ).values();
  }
  
  QSet<QString> ImageView::getTopics( const QSet<QString> &message_types, const QSet<QString> &message_sub_types,
                                      const QList<QString> &transports ) {
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics( topic_info );
    
    QSet<QString> all_topics;
    for ( ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++ ) {
      all_topics.insert( it->name.c_str());
    }
    
    QSet<QString> topics;
    for ( ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++ ) {
      if ( message_types.contains( it->datatype.c_str())) {
        QString topic = it->name.c_str();
        
        // add raw topic
        topics.insert( topic );
        
        // add transport specific sub-topics
        for ( QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++ ) {
          if ( all_topics.contains( topic + "/" + *jt )) {
            QString sub = topic + " " + *jt;
            topics.insert( sub );
          }
        }
      }
      if ( message_sub_types.contains( it->datatype.c_str())) {
        QString topic = it->name.c_str();
        int index = topic.lastIndexOf( "/" );
        if ( index != -1 ) {
          topic.replace( index, 1, " " );
          topics.insert( topic );
          //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
        }
      }
    }
    return topics;
  }
  
  void ImageView::selectTopic( const QString &topic ) {
    int index = ui_.topics_combo_box->findText( topic );
    if ( index == -1 ) {
      // add topic name to list if not yet in
      QString label( topic );
      label.replace( " ", "/" );
      ui_.topics_combo_box->addItem( label, QVariant( topic ));
      index = ui_.topics_combo_box->findText( topic );
    }
    ui_.topics_combo_box->setCurrentIndex( index );
  }
  
  void ImageView::onTopicChanged( int index ) {
    subscriber_.shutdown();
    
    // reset image on topic change
    ui_.image_frame->setImage( QImage());
    
    QStringList parts = ui_.topics_combo_box->itemData( index ).toString().split( " " );
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";
    
    std::string image_topic = topic.toStdString();
    std::string topic_camera_info;
    
    if ( !topic.isEmpty()) {
      
      for ( size_t i = image_topic.length() - 1; i >= 0; --i ) {
        if ( image_topic[i] == '/' ) {
          topic_camera_info.resize( i + 1 );
          std::copy( image_topic.begin(), image_topic.begin() + i + 1,
                     topic_camera_info.begin());
          break;
        }
      }
      
      topic_camera_info += "camera_info";
      
      image_transport::ImageTransport it( getNodeHandle());
      image_transport::TransportHints hints( transport.toStdString());
      
      try {
        subscriber_ = it.subscribe( topic.toStdString(), 1, &ImageView::callbackImage, this, hints );
        //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
      } catch (image_transport::TransportLoadException &e) {
        QMessageBox::warning( widget_, tr( "Loading image transport plugin failed" ), e.what());
      }
      
      sub_camera_info_ = getNodeHandle().subscribe( topic_camera_info, 1, &ImageView::callbackCameraInfo, this );
      //@ToDo: remove hardcoded
      sub_laser_ = getNodeHandle().subscribe( std::string( "/r0/laser0/scan" ), 1, &ImageView::callbackLaser, this );
    }
    
    
    onMousePublish( ui_.publish_click_location_check_box->isChecked());
  }
  
  void ImageView::onMousePublish( bool checked ) {
    std::string topicName;
    if ( pub_topic_custom_ ) {
      topicName = ui_.publish_click_location_topic_line_edit->text().toStdString();
    } else {
      if ( !subscriber_.getTopic().empty()) {
        topicName = subscriber_.getTopic() + "_mouse_left";
      } else {
        topicName = "mouse_left";
      }
      ui_.publish_click_location_topic_line_edit->setText( QString::fromStdString( topicName ));
    }
    
    //pub_fiducial_detection_ = getNodeHandle().advertise<marker_msgs::FiducialDetection>( "/fiducials", 1000 );
  }
  
  void ImageView::onMouseLeft( int x, int y ) {
    if ( !ui_.image_frame->getImage().isNull()) {
      geometry_msgs::Point clickCanvasLocation;
      // Publish click location in pixel coordinates
      clickCanvasLocation.x = round(
          (double) x / (double) ui_.image_frame->width() * (double) ui_.image_frame->getImage().width());
      clickCanvasLocation.y = round(
          (double) y / (double) ui_.image_frame->height() * (double) ui_.image_frame->getImage().height());
      clickCanvasLocation.z = 0;
      
      clickedPoints_.push_back( cv::Point2d( clickCanvasLocation.x, clickCanvasLocation.y ));
      //std::cout << "clicked x, y: " << clickedPoints_.back().x << ", " << clickedPoints_.back().y << std::endl;
      
      cv::circle( conversion_mat_, clickedPoints_.back(), 2, cv::Scalar( 0, 255, 0 ), 2 );
      QImage image( conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0],
                    QImage::Format_RGB888 );
      ui_.image_frame->setImage( image );
      
    }
  }
  
  void ImageView::onPubTopicChanged() {
    pub_topic_custom_ = !(ui_.publish_click_location_topic_line_edit->text().isEmpty());
    onMousePublish( ui_.publish_click_location_check_box->isChecked());
  }
  
  void ImageView::updateLaser2Map() {
    
    if ( measurement_laser_ && figure_local_->initialized()) {
      
      figure_local_->clear();
      
      {
        double c_left = cos( leftSplitAngle_ ) * restrict_left_laser_max_;
        double s_left = sin( leftSplitAngle_ ) * restrict_left_laser_max_;
        auto pleft = tuw::Point2D( c_left, s_left );
        double c_right = cos( rightSplitAngle_ ) * restrict_right_laser_max_;
        double s_right = sin( rightSplitAngle_ ) * restrict_right_laser_max_;
        auto pright = tuw::Point2D( c_right, s_right );
        
        figure_local_->line( tuw::Point2D( 0, 0 ), pleft, figure_local_->blue );
        figure_local_->line( tuw::Point2D( 0, 0 ), pright, figure_local_->blue );
      }
      
      bool is_rightmost = false;
      
      {
        std::lock_guard<std::mutex> lock( mutex_laser_ );
        for ( auto it_l = measurement_laser_->begin();
              it_l != measurement_laser_->end();
              ++it_l ) {
          if ( it_l->angle > leftSplitAngle_ && it_l->angle < rightSplitAngle_ &&
               it_l->range < restrict_left_laser_max_ ) {
            figure_local_->circle( it_l->end_point, 2, figure_local_->green );
            it_l->set_valid( true );
          } else {
            figure_local_->circle( it_l->end_point, 2, figure_local_->magenta );
            it_l->set_valid( false );
          }
        }
      }
      
      {
        std::lock_guard<std::mutex> lock( mutex_laser_ );
        cv::Mat view = figure_local_->view();
        //cv::imshow( "view", view );
        QImage image( view.data, view.cols, view.rows, view.step[0],
                      QImage::Format_RGB888 );
        ui_.laser_frame->setImage( image );
        ui_.laser_frame->setInnerFrameFixedSize( image.size());
        ui_.laser_frame->setEnabled( true );
        ui_.laser_frame->setVisible( true );
      }
      //std::cout << std::to_string( ui_.laser_frame->size().width()) << ", "
      //          << std::to_string( ui_.laser_frame->size().height()) << std::endl;
    }
  }
  
  void ImageView::onRefinePressed() {
    ui_.freeze_laser_checkbox->setChecked( false );
    ui_.freeze_image_checkbox->setChecked( false );
    ui_.laser_scan_checkbox->setChecked( false );
    clickedPoints_.clear();
  }
  
  bool ImageView::updateLaser2Image() {
    if ( measurement_image_ && measurement_image_->getCameraModel() && measurement_laser_ ) {
      
      laser2image_points_.clear();
      const auto T_WC = measurement_image_->getTfWorldSensor();
      const auto T_WL = measurement_laser_->getTfWorldSensor();
      const auto T_CL = T_WC.inverse() * T_WL;
      
      {
        std::lock_guard<std::mutex> lock( mutex_laser_ );
        laser2image_points_.resize( measurement_laser_->size());
        std::size_t i = 0;
        for ( auto beam_it = measurement_laser_->begin();
              beam_it != measurement_laser_->end();
              ++beam_it, ++i ) {
          Eigen::Vector4d laser_in_image =
              T_CL * Eigen::Vector4d( beam_it->end_point.x(), beam_it->end_point.y(), 0, 1 );
          laser_in_image = laser_in_image / laser_in_image[3];
          
          //const auto pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
          const cv::Point3d pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
          laser2image_points_[i] = measurement_image_->getCameraModel()->project3dToPixel( pnt3d );
        }
      }
      return true;
    }
    return false;
  }
  
  void ImageView::callbackLaser( const sensor_msgs::LaserScan &_laser ) {
    //@ToDo: here
    //std::cout << "laser cb" << std::endl;
    if ( !freeze_laser_scan_ ) {
      laser2image_points_.clear();
      
      tf::StampedTransform tf;
      tf.setIdentity();
      //if ( !getStaticTF( "/r0/base_link", _laser.header.frame_id, tf, false )) {
      //  //std::cout << "no laser gotten, return" << std::endl;
      //  return;
      //}
      
      std::lock_guard<std::mutex> lock( mutex_laser_ );
      measurement_laser_.reset( new tuw::LaserMeasurement( _laser, tf ));
      measurement_laser_->initFromScan();
    }
    //Auto check if measurement is there in method
    updateLaser2Map();
  }
  
  void ImageView::callbackCameraInfo( const sensor_msgs::CameraInfo::ConstPtr &_msg ) {
    //std::cout << "callbackCamerainfo" << std::endl;
    
    camera_model_.reset( new image_geometry::PinholeCameraModel());
    camera_model_->fromCameraInfo( _msg );
    if ( measurement_image_ && !measurement_image_->getCameraModel()) {
      measurement_image_->setCameraModel( camera_model_ );
    }
  }
  
  void ImageView::callbackImage( const sensor_msgs::Image::ConstPtr &msg ) {
    {
      std::lock_guard<std::mutex> lock( mutex_image_ );
      
      if ( !freeze_image_ ) {
        
        try {
          // First let cv_bridge do its magic
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::RGB8 );
          conversion_mat_ = cv_ptr->image;
          
          tf::StampedTransform tf;
          measurement_image_.reset( new tuw::ImageMeasurement( cv_ptr, tf ));
          if ( camera_model_ ) {
            measurement_image_->setCameraModel( camera_model_ );
          }
        }
        catch (cv_bridge::Exception &e) {
          try {
            // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare( msg );
            if ( msg->encoding == "CV_8UC3" ) {
              // assuming it is rgb
              conversion_mat_ = cv_ptr->image;
            } else if ( msg->encoding == "8UC1" ) {
              // convert gray to rgb
              cv::cvtColor( cv_ptr->image, conversion_mat_, CV_GRAY2RGB );
            } else {
              qWarning( "ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)",
                        msg->encoding.c_str(),
                        e.what());
              ui_.image_frame->setImage( QImage());
              return;
            }
          }
          catch (cv_bridge::Exception &e) {
            qWarning(
                "ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)",
                msg->encoding.c_str(), e.what());
            ui_.image_frame->setImage( QImage());
            return;
          }
        }
      }
      
    }
    drawImages();
  }
  
  void ImageView::onPublisherButton() {
    
    if ( clickedPoints_.size() == 4 && use_laser_scan_range_ ) {
      
      std::vector<cv::Point3f> object_points;
      tuw::Contour::Beam leftMostBeam;
      tuw::Contour::Beam rightMostBeam;
      {
        std::lock_guard<std::mutex> lock( mutex_laser_ );
        std::vector<tuw::Contour::Beam> laser_beams( measurement_laser_->size());
        auto end_it = std::copy_if( measurement_laser_->begin(), measurement_laser_->end(), laser_beams.begin(),
                                    []( const tuw::Contour::Beam &b ) {
                                      if ( b.is_valid()) {
                                        return true;
                                      }
                                      return false;
                                    } );
        laser_beams.resize( std::distance( laser_beams.begin(), end_it ));
        //@ToDo: check if this is correct!
        leftMostBeam = laser_beams.back();
        rightMostBeam = laser_beams.front();
      }
      
      size_t point_count = 0;
      for ( const cv::Point2d &pt : clickedPoints_ ) {
        
        cv::Point3d object_point;
        if ( point_count == 0 ) {
          object_point.x = leftMostBeam.end_point.x();
          object_point.y = leftMostBeam.end_point.y();
          object_point.z = 0;
        } else if ( point_count == 1 ) {
          object_point.x = leftMostBeam.end_point.x();
          object_point.y = leftMostBeam.end_point.y();
          object_point.z = 2.0f;
        } else if ( point_count == 2 ) {
          object_point.x = rightMostBeam.end_point.x();
          object_point.y = rightMostBeam.end_point.y();
          object_point.z = 2.0f;
        } else if ( point_count == 3 ) {
          object_point.x = rightMostBeam.end_point.x();
          object_point.y = rightMostBeam.end_point.y();
          object_point.z = 0.0f;
        }
        
        object_points.push_back( object_point );
        point_count++;
        
      }
      
      if ( !pnp_data_ ) {
        pnp_data_.reset( new PnPData());
        pnp_data_->K = measurement_image_->getCameraModel()->intrinsicMatrix();
        pnp_data_->D = measurement_image_->getCameraModel()->distortionCoeffs();
      }
      
      cv::Mat rv, tv;
      
      pnp_data_->image_points.insert( std::begin( pnp_data_->image_points ), std::begin( clickedPoints_ ),
                                      std::end( clickedPoints_ ));
      
      pnp_data_->object_points.insert( std::begin( pnp_data_->object_points ), std::begin( object_points ),
                                       std::end( object_points ));
      
      
      cv::solvePnP( pnp_data_->object_points, pnp_data_->image_points, pnp_data_->K,
                    pnp_data_->D, rv, tv );
      
      pnp_data_->T_LC = cv::Mat::eye( 4, 4, CV_64FC1);
      cv::Mat R33 = cv::Mat( pnp_data_->T_LC, cv::Rect( 0, 0, 3, 3 ));
      cv::Rodrigues( rv, R33 );
      
      for ( int i = 0; i < 3; ++i ) {
        pnp_data_->T_LC.at<double>( i, 3 ) = tv.at<double>( 0, i );
      }
      
      std::cout << "tv " << tv << std::endl;
      std::cout << "rv " << rv << std::endl;
      
      if ( measurement_laser_ ) {
        Eigen::Matrix4d T_WL = measurement_laser_->getTfWorldSensor();
        Eigen::Map<Eigen::Matrix4d> T_LC_e( reinterpret_cast<double *>(pnp_data_->T_LC.data));
        measurement_image_->setTfWorldSensor( T_WL * T_LC_e );
        updateLaser2Image();
      }
    }
  }
  
  bool ImageView::getStaticTF( const std::string &world_frame, const std::string &source_frame,
                               tf::StampedTransform &_pose, bool debug ) {
    
    std::string target_frame_id = source_frame;
    std::string source_frame_id = tf::resolve( "", world_frame );
    std::string key = target_frame_id + "->" + source_frame_id;
    
    if ( debug ) {
      ROS_INFO( "tf get: %s", key.c_str());
    }
    
    if ( !tfMap_[key] ) {
      try {
        
        listenerTF_.lookupTransform(
            source_frame_id, target_frame_id, ros::Time( 0 ), _pose );
        std::shared_ptr<tf::StampedTransform> tf_ref;
        tf_ref.reset( new tf::StampedTransform( _pose ));
        tfMap_[key] = std::move( tf_ref );
        
      } catch (tf::TransformException ex) {
        
        ROS_INFO( "getStaticTF" );
        ROS_ERROR( "%s", ex.what());
        ros::Duration( 1.0 ).sleep();
        return false;
        
      }
    }
    
    _pose = *tfMap_[key];
    
    if ( debug ) {
      std::cout << key << std::endl;
      std::cout << "tf get o: " << _pose.getOrigin().x() << ", " << _pose.getOrigin().y() << ", "
                << _pose.getOrigin().z()
                << std::endl;
      std::cout << "tf get r: " << _pose.getRotation().x() << ", " << _pose.getRotation().y() << ", "
                << _pose.getRotation().z() << ", " << _pose.getRotation().w() << std::endl;
    }
    
    return true;
  }
}

PLUGINLIB_EXPORT_CLASS( rqt_image_view::ImageView, rqt_gui_cpp::Plugin )
