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

namespace rqt_image_view {
  
  ImageView::ImageView()
      : rqt_gui_cpp::Plugin(), widget_( 0 ), num_gridlines_( 0 ), rotate_state_( ROTATE_0 ) {
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
    
    ui_.zoom_1_push_button->setIcon( QIcon::fromTheme( "zoom-original" ));
    connect( ui_.zoom_1_push_button, SIGNAL( toggled( bool )), this, SLOT( onZoom1( bool )) );
    
    connect( ui_.dynamic_range_check_box, SIGNAL( toggled( bool )), this, SLOT( onDynamicRange( bool )) );
    
    ui_.save_as_image_push_button->setIcon( QIcon::fromTheme( "document-save-as" ));
    connect( ui_.save_as_image_push_button, SIGNAL( pressed()), this, SLOT( saveImage()) );
    
    connect( ui_.num_gridlines_spin_box, SIGNAL( valueChanged( int )), this, SLOT( updateNumGridlines()) );
    
    // set topic name if passed in as argument
    const QStringList &argv = context.argv();
    if ( !argv.empty()) {
      arg_topic_name = argv[0];
      selectTopic( arg_topic_name );
    }
    pub_topic_custom_ = false;
    
    //ui_.image_frame->setOuterLayout( ui_.image_layout );
    //ui_.laser_frame->setOuterLayout( ui_.image_layout );
    ui_.laser_frame->setImage( QImage());
    
    QRegExp rx(
        "([a-zA-Z/][a-zA-Z0-9_/]*)?" ); //see http://www.ros.org/wiki/ROS/Concepts#Names.Valid_Names (but also accept an empty field)
    ui_.publish_click_location_topic_line_edit->setValidator( new QRegExpValidator( rx, this ));
    connect( ui_.publish_click_location_check_box, SIGNAL( toggled( bool )), this, SLOT( onMousePublish( bool )) );
    connect( ui_.image_frame, SIGNAL( mouseLeft( int, int )), this, SLOT( onMouseLeft( int, int )) );
    connect( ui_.publish_click_location_topic_line_edit, SIGNAL( editingFinished()), this, SLOT( onPubTopicChanged()) );
    
    connect( ui_.smooth_image_check_box, SIGNAL( toggled( bool )), ui_.image_frame,
             SLOT( onSmoothImageChanged( bool )) );
    
    connect( ui_.rotate_left_push_button, SIGNAL( clicked( bool )), this, SLOT( onRotateLeft()) );
    connect( ui_.rotate_right_push_button, SIGNAL( clicked( bool )), this, SLOT( onRotateRight()) );
    connect( ui_.laser_scan_checkbox, SIGNAL( toggled( bool )), this, SLOT( onLaserScanBoxToggle( bool )) );
    connect( ui_.freeze_laser_checkbox, SIGNAL( toggled( bool )), this, SLOT( onFreezeLaserBoxToggle( bool )) );
    
    ui_.laser_scan_checkbox->toggled(false);
    ui_.freeze_laser_checkbox->toggled(false);
    
    ui_.sliderLeftLaser->setMinimum( 0 );
    ui_.sliderLeftLaser->setMaximum( 100 );
    ui_.sliderLeftLaser->setValue( 0 );
    ui_.sliderRightLaser->setMinimum( 0 );
    ui_.sliderRightLaser->setMaximum( 100 );
    ui_.sliderRightLaser->setValue( 100 );
    
    connect( ui_.sliderLeftLaser, SIGNAL( valueChanged( int )), this, SLOT( onLeftSliderValChanged( int )) );
    connect( ui_.sliderRightLaser, SIGNAL( valueChanged( int )), this, SLOT( onRightSliderValChanged( int )) );
    
    leftSplitAngle_ = -M_PI / 2.0;
    rightSplitAngle_ = M_PI / 2.0;
    
    //connect( ui_.laser_frame, ,this, SLOT(onLaserTopichChanged() ));
    
    // Make sure we have enough space for "XXX °"
    ui_.rotate_label->setMinimumWidth(
        ui_.rotate_label->fontMetrics().width( "XXX°" )
    );
    
    hide_toolbar_action_ = new QAction( tr( "Hide toolbar" ), this );
    hide_toolbar_action_->setCheckable( true );
    ui_.image_frame->addAction( hide_toolbar_action_ );
    connect( hide_toolbar_action_, SIGNAL( toggled( bool )), this, SLOT( onHideToolbarChanged( bool )) );
    
    clickedPoints_ = boost::circular_buffer<cv::Point2d>( 4 );
    
    figure_local_.reset( new tuw::Figure( "laser2map" ));
    figure_local_->init( 500, 500, -5, 5, -5, 5, M_PI / 2.0, 1, 1 );
  }
  
  void ImageView::shutdownPlugin() {
    subscriber_.shutdown();
    pub_mouse_left_.shutdown();
  }
  
  void ImageView::onFreezeLaserBoxToggle( bool val ) {
    freeze_laser_scan_ = val;
  }
  
  void ImageView::onLaserScanBoxToggle( bool val ) {
    use_laser_scan_range_ = val;
  }
  
  void ImageView::onLeftSliderValChanged( int val ) {
    if ( measurement_laser_ ) {
      double angle_min = measurement_laser_->getLaser().angle_min;
      double angle_max = measurement_laser_->getLaser().angle_max;
      rightSplitAngle_ =
          ((static_cast<double>(val) / 100.0) * (angle_max - angle_min)) + angle_min;
      std::cout << "left angle " << leftSplitAngle_ << std::endl;
    }
  }
  
  void ImageView::onRightSliderValChanged( int val ) {
    if ( measurement_laser_ ) {
      double angle_min = measurement_laser_->getLaser().angle_min;
      double angle_max = measurement_laser_->getLaser().angle_max;
      leftSplitAngle_ =
          ((static_cast<double>(val) / 100.0) * (angle_max - angle_min)) + angle_min;
      std::cout << "right angle " << rightSplitAngle_ << std::endl;
    }
  }
  
  void ImageView::saveSettings( qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings ) const {
    QString topic = ui_.topics_combo_box->currentText();
    //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
    instance_settings.setValue( "topic", topic );
    instance_settings.setValue( "zoom1", ui_.zoom_1_push_button->isChecked());
    instance_settings.setValue( "dynamic_range", ui_.dynamic_range_check_box->isChecked());
    instance_settings.setValue( "max_range", ui_.max_range_double_spin_box->value());
    instance_settings.setValue( "publish_click_location", ui_.publish_click_location_check_box->isChecked());
    instance_settings.setValue( "mouse_pub_topic", ui_.publish_click_location_topic_line_edit->text());
    instance_settings.setValue( "toolbar_hidden", hide_toolbar_action_->isChecked());
    instance_settings.setValue( "num_gridlines", ui_.num_gridlines_spin_box->value());
    instance_settings.setValue( "smooth_image", ui_.smooth_image_check_box->isChecked());
    instance_settings.setValue( "rotate", rotate_state_ );
    //instance_settings.setValue( "laser_box", ui_.laser_scan_checkbox->isChecked());
  }
  
  void ImageView::restoreSettings( const qt_gui_cpp::Settings &plugin_settings,
                                   const qt_gui_cpp::Settings &instance_settings ) {
    bool zoom1_checked = instance_settings.value( "zoom1", false ).toBool();
    ui_.zoom_1_push_button->setChecked( zoom1_checked );
    bool dynamic_range_checked = instance_settings.value( "dynamic_range", false ).toBool();
    ui_.dynamic_range_check_box->setChecked( dynamic_range_checked );
    
    double max_range = instance_settings.value( "max_range", ui_.max_range_double_spin_box->value()).toDouble();
    ui_.max_range_double_spin_box->setValue( max_range );
    
    num_gridlines_ = instance_settings.value( "num_gridlines", ui_.num_gridlines_spin_box->value()).toInt();
    ui_.num_gridlines_spin_box->setValue( num_gridlines_ );
    
    QString topic = instance_settings.value( "topic", "" ).toString();
    // don't overwrite topic name passed as command line argument
    if ( !arg_topic_name.isEmpty()) {
      arg_topic_name = "";
    } else {
      //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
      selectTopic( topic );
    }
    
    bool publish_click_location = instance_settings.value( "publish_click_location", false ).toBool();
    ui_.publish_click_location_check_box->setChecked( publish_click_location );
    
    QString pub_topic = instance_settings.value( "mouse_pub_topic", "" ).toString();
    ui_.publish_click_location_topic_line_edit->setText( pub_topic );
    
    bool toolbar_hidden = instance_settings.value( "toolbar_hidden", false ).toBool();
    hide_toolbar_action_->setChecked( toolbar_hidden );
    
    bool smooth_image_checked = instance_settings.value( "smooth_image", false ).toBool();
    ui_.smooth_image_check_box->setChecked( smooth_image_checked );
    
    rotate_state_ = static_cast<RotateState>(instance_settings.value( "rotate", 0 ).toInt());
    if ( rotate_state_ >= ROTATE_STATE_COUNT )
      rotate_state_ = ROTATE_0;
    syncRotateLabel();
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
        //qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());
        
        // add transport specific sub-topics
        for ( QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++ ) {
          if ( all_topics.contains( topic + "/" + *jt )) {
            QString sub = topic + " " + *jt;
            topics.insert( sub );
            //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
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
      //std::cout << "camera info topic " << topic_camera_info << std::endl;
      
      image_transport::ImageTransport it( getNodeHandle());
      image_transport::TransportHints hints( transport.toStdString());
      
      //std::cout << "subscribing to " << topic.toStdString() << std::endl;
      //std::cout << "subscribing to " << topic_camera_info << std::endl;
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
  
  void ImageView::onZoom1( bool checked ) {
    if ( checked ) {
      if ( ui_.image_frame->getImage().isNull()) {
        return;
      }
      ui_.image_frame->setInnerFrameFixedSize( ui_.image_frame->getImage().size());
    } else {
      ui_.image_frame->setInnerFrameMinimumSize( QSize( 80, 60 ));
      ui_.image_frame->setMaximumSize( QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
      widget_->setMinimumSize( QSize( 80, 60 ));
      widget_->setMaximumSize( QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    }
  }
  
  void ImageView::onDynamicRange( bool checked ) {
    ui_.max_range_double_spin_box->setEnabled( !checked );
  }
  
  void ImageView::updateNumGridlines() {
    num_gridlines_ = ui_.num_gridlines_spin_box->value();
  }
  
  void ImageView::saveImage() {
    // take a snapshot before asking for the filename
    QImage img = ui_.image_frame->getImageCopy();
    
    QString file_name = QFileDialog::getSaveFileName( widget_, tr( "Save as image" ), "image.png",
                                                      tr( "Image (*.bmp *.jpg *.png *.tiff)" ));
    if ( file_name.isEmpty()) {
      return;
    }
    
    img.save( file_name );
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
    
    if ( checked ) {
      pub_mouse_left_ = getNodeHandle().advertise<geometry_msgs::Point>( topicName, 1000 );
    } else {
      pub_mouse_left_.shutdown();
    }
    
    pub_fiducial_detection_ = getNodeHandle().advertise<marker_msgs::FiducialDetection>( "/fiducials", 1000 );
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
      
      geometry_msgs::Point clickLocation = clickCanvasLocation;
      
      clickedPoints_.push_back( cv::Point2d( clickCanvasLocation.x, clickCanvasLocation.y ));
      std::cout << "clicked x, y: " << clickedPoints_.back().x << ", " << clickedPoints_.back().y << std::endl;
      
      cv::circle( conversion_mat_, clickedPoints_.back(), 2, cv::Scalar( 0, 255, 0 ), 2 );
      QImage image( conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0],
                    QImage::Format_RGB888 );
      ui_.image_frame->setImage( image );
      //switch (rotate_state_) {
      //  case ROTATE_90:
      //    clickLocation.x = clickCanvasLocation.y;
      //    clickLocation.y = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
      //    break;
      //  case ROTATE_180:
      //    clickLocation.x = ui_.image_frame->getImage().width() - clickCanvasLocation.x;
      //    clickLocation.y = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
      //    break;
      //  case ROTATE_270:
      //    clickLocation.x = ui_.image_frame->getImage().height() - clickCanvasLocation.y;
      //    clickLocation.y = clickCanvasLocation.x;
      //    break;
      //  default:
      //    break;
      //}
      
      pub_mouse_left_.publish( clickLocation );
    }
  }
  
  void ImageView::onPubTopicChanged() {
    pub_topic_custom_ = !(ui_.publish_click_location_topic_line_edit->text().isEmpty());
    onMousePublish( ui_.publish_click_location_check_box->isChecked());
  }
  
  void ImageView::onHideToolbarChanged( bool hide ) {
    ui_.toolbar_widget->setVisible( !hide );
  }
  
  void ImageView::onRotateLeft() {
    int m = rotate_state_ - 1;
    if ( m < 0 )
      m = ROTATE_STATE_COUNT - 1;
    
    rotate_state_ = static_cast<RotateState>(m);
    syncRotateLabel();
  }
  
  void ImageView::onRotateRight() {
    rotate_state_ = static_cast<RotateState>((rotate_state_ + 1) % ROTATE_STATE_COUNT);
    syncRotateLabel();
  }
  
  void ImageView::syncRotateLabel() {
    switch ( rotate_state_ ) {
      default:
      case ROTATE_0:
        ui_.rotate_label->setText( "0°" );
        break;
      case ROTATE_90:
        ui_.rotate_label->setText( "90°" );
        break;
      case ROTATE_180:
        ui_.rotate_label->setText( "180°" );
        break;
      case ROTATE_270:
        ui_.rotate_label->setText( "270°" );
        break;
    }
  }
  
  void ImageView::invertPixels( int x, int y ) {
    // Could do 255-conversion_mat_.at<cv::Vec3b>(cv::Point(x,y))[i], but that doesn't work well on gray
    cv::Vec3b &pixel = conversion_mat_.at<cv::Vec3b>( cv::Point( x, y ));
    if ( pixel[0] + pixel[1] + pixel[2] > 3 * 127 )
      pixel = cv::Vec3b( 0, 0, 0 );
    else
      pixel = cv::Vec3b( 255, 255, 255 );
  }
  
  QList<int> ImageView::getGridIndices( int size ) const {
    QList<int> indices;
    
    // the spacing between adjacent grid lines
    float grid_width = 1.0f * size / (num_gridlines_ + 1);
    
    // select grid line(s) closest to the center
    float index;
    if ( num_gridlines_ % 2 )  // odd
    {
      indices.append( size / 2 );
      // make the center line 2px wide in case of an even resolution
      if ( size % 2 == 0 )  // even
        indices.append( size / 2 - 1 );
      index = 1.0f * (size - 1) / 2;
    } else  // even
    {
      index = grid_width * (num_gridlines_ / 2);
      // one grid line before the center
      indices.append( round( index ));
      // one grid line after the center
      indices.append( size - 1 - round( index ));
    }
    
    // add additional grid lines from the center to the border of the image
    int lines = (num_gridlines_ - 1) / 2;
    while ( lines > 0 ) {
      index -= grid_width;
      indices.append( round( index ));
      indices.append( size - 1 - round( index ));
      lines--;
    }
    
    return indices;
  }
  
  void ImageView::overlayGrid() {
    // vertical gridlines
    QList<int> columns = getGridIndices( conversion_mat_.cols );
    for ( QList<int>::const_iterator x = columns.begin(); x != columns.end(); ++x ) {
      for ( int y = 0; y < conversion_mat_.rows; ++y ) {
        invertPixels( *x, y );
      }
    }
    
    // horizontal gridlines
    QList<int> rows = getGridIndices( conversion_mat_.rows );
    for ( QList<int>::const_iterator y = rows.begin(); y != rows.end(); ++y ) {
      for ( int x = 0; x < conversion_mat_.cols; ++x ) {
        invertPixels( x, *y );
      }
    }
  }
  
  void ImageView::updateLaser2Map() {
    if ( measurement_laser_ ) {
      figure_local_->clear();
      
      {
        double c_left = cos( leftSplitAngle_ ) * 10.0;
        double s_left = sin( leftSplitAngle_ ) * 10.0;
        auto pleft = tuw::Point2D( c_left, s_left );
        double c_right = cos( rightSplitAngle_ ) * 10.0;
        double s_right = sin( rightSplitAngle_ ) * 10.0;
        auto pright = tuw::Point2D( c_right, s_right );
        
        figure_local_->line( tuw::Point2D( 0, 0 ), pleft, figure_local_->blue );
        figure_local_->line( tuw::Point2D( 0, 0 ), pright, figure_local_->blue );
      }
      
      bool is_rightmost = false;
      for ( auto it_l = measurement_laser_->begin();
            it_l != measurement_laser_->end();
            ++it_l ) {
        if ( it_l->angle > leftSplitAngle_ && it_l->angle < rightSplitAngle_ ) {
          if ( !is_rightmost ) {
            figure_local_->circle( it_l->end_point, 2, figure_local_->red );
            is_rightmost = true;
          } else {
            figure_local_->circle( it_l->end_point, 2, figure_local_->green );
          }
        } else {
          figure_local_->circle( it_l->end_point, 2, figure_local_->magenta );
          it_l->set_valid( false );
        }
      }
      
      cv::Mat view = figure_local_->view();
      //cv::imshow( "view", view );
      QImage image( view.data, view.cols, view.rows, view.step[0],
                    QImage::Format_RGB888 );
      ui_.laser_frame->setImage( image );
      ui_.laser_frame->setInnerFrameFixedSize( image.size());
      ui_.laser_frame->setEnabled( true );
      ui_.laser_frame->setVisible( true );
      //std::cout << std::to_string( ui_.laser_frame->size().width()) << ", "
      //          << std::to_string( ui_.laser_frame->size().height()) << std::endl;
    }
  }
  
  bool ImageView::updateLaser2Image() {
    if ( measurement_image_ && measurement_image_->getCameraModel() && measurement_laser_ ) {
      
      laser2image_points_.clear();
      const auto T_WC = measurement_image_->getTfWorldSensor();
      const auto T_WL = measurement_laser_->getTfWorldSensor();
      const auto T_CL = T_WC.inverse() * T_WL;
      
      std::size_t i = 0;
      laser2image_points_.resize( measurement_laser_->size());
      
      for ( auto beam_it = measurement_laser_->begin();
            beam_it != measurement_laser_->end();
            ++beam_it, ++i ) {
        Eigen::Vector4d laser_in_image = T_CL * Eigen::Vector4d( beam_it->end_point.x(), beam_it->end_point.y(), 0, 1 );
        laser_in_image = laser_in_image / laser_in_image[3];
        
        //const auto pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
        const cv::Point3d pnt3d = cv::Point3d( laser_in_image[0], laser_in_image[1], laser_in_image[2] );
        laser2image_points_[i] = measurement_image_->getCameraModel()->project3dToPixel( pnt3d );
      }
      
      return true;
    }
    
    return false;
  }
  
  void ImageView::callbackLaser( const sensor_msgs::LaserScan &_laser ) {
    //@ToDo: here
    //std::cout << "laser cb" << std::endl;
    laser2image_points_.clear();
    
    tf::StampedTransform tf;
    tf.setIdentity();
    //if ( !getStaticTF( "/r0/base_link", _laser.header.frame_id, tf, false )) {
    //  //std::cout << "no laser gotten, return" << std::endl;
    //  return;
    //}
    
    measurement_laser_.reset( new tuw::LaserMeasurement( _laser, tf ));
    measurement_laser_->initFromScan();
    
    updateLaser2Map();
  }
  
  void ImageView::callbackCameraInfo( const sensor_msgs::CameraInfo::ConstPtr &_msg ) {
    //std::cout << "callbackCamerainfo" << std::endl;
    
    camera_model_.reset( new image_geometry::PinholeCameraModel());
    camera_model_->fromCameraInfo( _msg );
    if ( measurement_image_ && !measurement_image_->getCameraModel()) {
      measurement_image_->setCameraModel( camera_model_ );
    }
    //std::cout << "callbackCamerainfo end" << std::endl;
  }
  
  void ImageView::callbackImage( const sensor_msgs::Image::ConstPtr &msg ) {
    //std::cout << "cb image" << std::endl;
    try {
      // First let cv_bridge do its magic
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::RGB8 );
      conversion_mat_ = cv_ptr->image;
      
      tf::StampedTransform tf;
      measurement_image_.reset( new tuw::ImageMeasurement( cv_ptr, tf ));
      if ( camera_model_ ) {
        measurement_image_->setCameraModel( camera_model_ );
      }
      //if ( getStaticTF( "/r0/base_link", msg->header.frame_id.c_str(), tf, false )) {
      //  measurement_image_.reset( new tuw::ImageMeasurement( cv_ptr, tf ));
      //}
      
      if ( num_gridlines_ > 0 )
        overlayGrid();
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
        } else if ( msg->encoding == "16UC1" || msg->encoding == "32FC1" ) {
          // scale / quantify
          double min = 0;
          double max = ui_.max_range_double_spin_box->value();
          if ( msg->encoding == "16UC1" ) max *= 1000;
          if ( ui_.dynamic_range_check_box->isChecked()) {
            // dynamically adjust range based on min/max in image
            cv::minMaxLoc( cv_ptr->image, &min, &max );
            if ( min == max ) {
              // completely homogeneous images are displayed in gray
              min = 0;
              max = 2;
            }
          }
          cv::Mat img_scaled_8u;
          cv::Mat( cv_ptr->image - min ).convertTo( img_scaled_8u, CV_8UC1, 255. / (max - min));
          cv::cvtColor( img_scaled_8u, conversion_mat_, CV_GRAY2RGB );
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
    
    // Handle rotation
    switch ( rotate_state_ ) {
      case ROTATE_90: {
        cv::Mat tmp;
        cv::transpose( conversion_mat_, tmp );
        cv::flip( tmp, conversion_mat_, 1 );
        break;
      }
      case ROTATE_180: {
        cv::Mat tmp;
        cv::flip( conversion_mat_, tmp, -1 );
        conversion_mat_ = tmp;
        break;
      }
      case ROTATE_270: {
        cv::Mat tmp;
        cv::transpose( conversion_mat_, tmp );
        cv::flip( tmp, conversion_mat_, 0 );
        break;
      }
      default:
        break;
    }
    
    if ( clickedPoints_.size()) {
      //std::cout << "clickedPoints" << std::endl;
      for ( int i = 0; i < clickedPoints_.size(); ++i ) {
        cv::circle( conversion_mat_, clickedPoints_[i], 2, cv::Scalar( 0, 255, 0 ), 2 );
        if ( measurement_image_ && measurement_image_->getCameraModel()) {
          cv::Point3d pt3d = measurement_image_->getCameraModel()->projectPixelTo3dRay( clickedPoints_[i] );
          //cv::putText( conversion_mat_,
          //             ("(" + std::to_string( pt3d.x ) + ", " + std::to_string( pt3d.y ) + std::to_string( pt3d.z )
          //              + ")"), clickedPoints_[i] + cv::Point2d( 1, 0 ), CV_FONT_HERSHEY_COMPLEX, 1,
          //             cv::Scalar( 0, 255, 0 ));
        }
      }
      //std::cout << "end" << std::endl;
    }
    
    if ( laser2image_points_.size()) {
      //std::cout << "laser2image " << std::endl;
      for ( size_t i = 0; i < laser2image_points_.size(); ++i ) {
        cv::circle( conversion_mat_, laser2image_points_[i], 2, cv::Scalar( 255, 0, 0 ), 2 );
      }
      //std::cout << "laser2image circle" << std::endl;
    }
    
    // image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
    QImage image( conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0],
                  QImage::Format_RGB888 );
    ui_.image_frame->setImage( image );
    
    if ( !ui_.zoom_1_push_button->isEnabled()) {
      ui_.zoom_1_push_button->setEnabled( true );
    }
    // Need to update the zoom 1 every new image in case the image aspect ratio changed,
    // though could check and see if the aspect ratio changed or not.
    onZoom1( ui_.zoom_1_push_button->isChecked());
    
    //std::cout << "end image cb" << std::endl;
    publishFiducials();
  }
  
  void ImageView::publishFiducials() {
    if ( clickedPoints_.size() == 4 && use_laser_scan_range_ ) {
      marker_msgs::FiducialDetection fd;
      
      if ( measurement_image_->getCameraModel()) {
        std::cout << "Has camera model" << std::endl;
      } else {
        std::cout << "Has no camera model" << std::endl;
      }

#ifdef ENABLE_PUBLISHING
      cv::Matx33d K = measurement_image_->getCameraModel()->intrinsicMatrix();
      cv::Mat D = measurement_image_->getCameraModel()->distortionCoeffs();
      
      size_t k = 0;
      for ( size_t i = 0; i < K.rows; ++i ) {
        for ( size_t j = 0; j < K.cols; ++j ) {
          fd.camera_k[k] = K( i, j );
          k++;
        }
      }
      
      for ( int i = 0; i < 5; ++i ) {
        fd.camera_d.push_back( 0 );
      }
      
      for ( size_t i = 0; i < K.rows; ++i ) {
        for ( size_t j = 0; j < K.cols; ++j ) {
          //fd.camera_d.push_back( D.at<double>( i, j ));
          //std::cout << D.at<double>( i, j ) << ", " << std::endl;
        }
      }
      
      fd.fiducial.resize( 1 );
      fd.fiducial[0].ids = {0};
      fd.fiducial[0].ids_confidence = {1.0};
      
      size_t point_count = 0;
      //Fixed order bottom left, top left, top right, bottom right
      for ( const cv::Point2d &pt : clickedPoints_ ) {
        geometry_msgs::Point image_point;
        image_point.x = pt.x;
        image_point.y = pt.y;
        fd.fiducial.back().image_points.push_back( image_point );
        
        geometry_msgs::Point object_point;
        
        if ( point_count == 0 ) {
          object_point.x = 0;
          object_point.y = 0;
          object_point.z = 0;
        } else if ( point_count == 1 ) {
          object_point.x = 0;
          object_point.y = 0;
          object_point.z = 2.0f;
        } else if ( point_count == 2 ) {
          object_point.x = 0;
          object_point.y = 0.9f;
          object_point.z = 2.0f;
        } else if ( point_count == 3 ) {
          object_point.x = 0;
          object_point.y = 0.9f;
          object_point.z = 0.0f;
        }
        
        fd.fiducial.back().object_points.push_back( object_point );
        point_count++;
      }
      
      pub_fiducial_detection_.publish( fd );
    //}
#else
      std::vector<cv::Point3f> object_points;
      std::vector<tuw::Contour::Beam> laser_beams( measurement_laser_->size());
      std::cout << "getting beams" << std::endl;
      std::cout << "laser beams " << measurement_laser_->size() << std::endl;
      auto end_it = std::copy_if( measurement_laser_->begin(), measurement_laser_->end(), laser_beams.begin(),
                                  []( const tuw::Contour::Beam &b ) {
                                    if ( b.is_valid()) {
                                      return true;
                                    }
                                    return false;
                                  } );
      laser_beams.resize( std::distance( laser_beams.begin(), end_it ));
      std::cout << "got beams" << std::endl;
      tuw::Contour::Beam leftMost = laser_beams.back();
      tuw::Contour::Beam rightMost = laser_beams.front();
      
      size_t point_count = 0;
      for ( const cv::Point2d &pt : clickedPoints_ ) {
        
        cv::Point3d object_point;
        if ( point_count == 0 ) {
          object_point.x = leftMost.end_point.x();
          object_point.y = leftMost.end_point.y();
          object_point.z = 0;
        } else if ( point_count == 1 ) {
          object_point.x = leftMost.end_point.x();
          object_point.y = leftMost.end_point.y();
          object_point.z = 2.0f;
        } else if ( point_count == 2 ) {
          object_point.x = rightMost.end_point.x();
          object_point.y = rightMost.end_point.y();
          object_point.z = 2.0f;
        } else if ( point_count == 3 ) {
          object_point.x = rightMost.end_point.x();
          object_point.y = rightMost.end_point.y();
          object_point.z = 0.0f;
        }
        
        object_points.push_back( object_point );
        point_count++;
        
      }
      
      cv::Mat rv, tv;
      std::vector<cv::Point2f> image_pnts;
      image_pnts.resize( clickedPoints_.size());
      std::copy( clickedPoints_.begin(), clickedPoints_.end(), image_pnts.begin());
      cv::Matx33d Kmat = measurement_image_->getCameraModel()->intrinsicMatrix();
      cv::Mat Dmat = measurement_image_->getCameraModel()->distortionCoeffs();
      
      cv::solvePnP( object_points, image_pnts, Kmat,
                    Dmat, rv, tv );
      
      //This is the tf from the door to the camera
      cv::Mat T_LC = cv::Mat::eye( 4, 4, CV_64FC1);
      cv::Mat R33 = cv::Mat( T_LC, cv::Rect( 0, 0, 3, 3 ));
      cv::Rodrigues( rv, R33 );
      //std::cout << "size tv " << tv.size() << std::endl;
      //std::cout << "size rv " << rv.size() << std::endl;
      //for ( int i = 0; i < R33.rows; ++i ) {
      //  for ( int j = 0; j < R33.cols; ++j ) {
      //    m.at<double>( i, j ) = R33.at<double>( i, j );
      //  }
      //}
      
      for ( int i = 0; i < 3; ++i ) {
        T_LC.at<double>( i, 3 ) = tv.at<double>( 0, i );
      }
      
      std::cout << T_LC << std::endl;
      std::cout << "tv " << tv << std::endl;
      std::cout << "rv " << rv << std::endl;
      
      if ( measurement_laser_ ) {
        measurement_laser_->setTfWorldSensor( T_LC );
        updateLaser2Image();
      }

#endif
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
