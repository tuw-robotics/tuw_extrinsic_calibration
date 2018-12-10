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

#include <tuw_extrinsic_camera/image_view.h>

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

namespace tuw_extrinsic_camera {

  ImageView::ImageView()
      : rqt_gui_cpp::Plugin(), widget_(0), num_gridlines_(0), tf_buffer_(ros::Duration(100, 0)),
        tf_listener_(tf_buffer_) {
    setObjectName("ImageView");
  }

  void ImageView::initPlugin(qt_gui_cpp::PluginContext &context) {
    widget_ = new QWidget();
    ui_.setupUi(widget_);

    if (context.serialNumber() > 1) {
      widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    updateTopicList();
    ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
    connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

    connect(ui_.undo_button, SIGNAL(pressed()), this, SLOT(onUndo()));

    ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
    connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

    updateLaserTopicList();
    ui_.refresh_laser_topics_button->setIcon(QIcon::fromTheme("view-refresh"));
    connect(ui_.refresh_laser_topics_button, SIGNAL(pressed()), this, SLOT(updateLaserTopicList()));

    //ui_.save_as_image_push_button->setIcon( QIcon::fromTheme( "document-save-as" ));
    //connect( ui_.save_as_image_push_button, SIGNAL( pressed()), this, SLOT( saveImage()) );

    // set topic name if passed in as argument
    const QStringList &argv = context.argv();
    if (!argv.empty()) {
      arg_topic_name = argv[0];
      selectTopic(arg_topic_name);
    }

    ui_.image_frame->setImage(QImage());
    ui_.laser_frame->setImage(QImage());

    QRegExp rx(
        "([a-zA-Z/][a-zA-Z0-9_/]*)?"); //see http://www.ros.org/wiki/ROS/Concepts#Names.Valid_Names (but also accept an empty field)
    ui_.pub_topic_textfield->setValidator(new QRegExpValidator(rx, this));
    connect(ui_.pub_topic_textfield, SIGNAL(editingFinished()), this, SLOT(onPubTopicChanged()));

    ui_.base_link_textfield->setValidator(new QRegExpValidator(rx, this));
    connect(ui_.base_link_textfield, SIGNAL(editingFinished()), this, SLOT(onBaseTFTopicChanged()));

    connect(ui_.image_frame, SIGNAL(mouseLeft(int, int)), this, SLOT(onMouseLeft(int, int)));

    //connect( ui_.publish_click_location_topic_line_edit, SIGNAL( editingFinished()), this, SLOT( onPubTopicChanged()));
    connect(ui_.zoom_in_button, SIGNAL(pressed()), this, SLOT(onZoomIn()));
    connect(ui_.zoom_out_button, SIGNAL(pressed()), this, SLOT(onZoomOut()));

    connect(ui_.laser_scan_checkbox, SIGNAL(toggled(bool)), this, SLOT(onLaserScanBoxToggle(bool)));
    connect(ui_.freeze_laser_checkbox, SIGNAL(toggled(bool)), this, SLOT(onFreezeLaserBoxToggle(bool)));
    connect(ui_.freeze_image_checkbox, SIGNAL(toggled(bool)), this, SLOT(onFreezeImageBoxToggle(bool)));
    connect(ui_.publisher_button, SIGNAL(pressed()), this, SLOT(onPublisherButton()));
    connect(ui_.refine_button, SIGNAL(pressed()), this, SLOT(onRefinePressed()));

    ui_.laser_scan_checkbox->toggled(false);
    ui_.freeze_laser_checkbox->toggled(false);
    ui_.freeze_image_checkbox->toggled(false);

    ui_.sliderLeftLaser->setMinimum(sliders_.ui_slider_angle_min_);
    ui_.sliderRightLaser->setMinimum(sliders_.ui_slider_angle_min_);

    ui_.sliderLeftLaser->setMaximum(sliders_.ui_slider_angle_max_);
    ui_.sliderRightLaser->setMaximum(sliders_.ui_slider_angle_max_);

    connect(ui_.sliderLeftLaser, SIGNAL(valueChanged(int)), this, SLOT(onLeftSliderValChanged(int)));
    connect(ui_.sliderRightLaser, SIGNAL(valueChanged(int)), this, SLOT(onRightSliderValChanged(int)));
    connect(ui_.sliderLeftLaserD, SIGNAL(valueChanged(int)), this, SLOT(onLeftDistanceSliderValChanged(int)));
    connect(ui_.sliderRightLaserD, SIGNAL(valueChanged(int)), this, SLOT(onRightDistanceSliderValChanged(int)));

    ui_.sliderLeftLaser->setValue(sliders_.ui_slider_angle_max_);
    onLeftSliderValChanged(sliders_.ui_slider_angle_max_);
    ui_.sliderRightLaser->setValue(sliders_.ui_slider_angle_min_);
    onRightSliderValChanged(sliders_.ui_slider_angle_max_);

    ui_.sliderRightLaserD->setMinimum(sliders_.ui_slider_dist_min_);
    ui_.sliderLeftLaserD->setMinimum(sliders_.ui_slider_dist_min_);

    ui_.sliderLeftLaserD->setMaximum(sliders_.ui_slider_dist_max_);
    ui_.sliderRightLaserD->setMaximum(sliders_.ui_slider_dist_max_);

    ui_.sliderLeftLaserD->setValue(sliders_.ui_slider_dist_max_ * 0.5);
    ui_.sliderRightLaserD->setValue(sliders_.ui_slider_dist_max_ * 0.5);

    image_properties_.clickedPoints_ = boost::circular_buffer<cv::Point2d>(4);

    laser_properties_.figure_local_.reset(new tuw::Figure("laser2map"));
    laser_properties_.figure_local_->init(700, 700, -0.5, 8, -5, 5, M_PI / 2.0, 1, 1);
  }

  void ImageView::onUndo() {
    lock();
    if (pnp_data_) {
      if (pnp_data_->image_points.size() >= 4) {
        pnp_data_->image_points.resize(pnp_data_->image_points.size() - 4);
        pnp_data_->object_points.resize(pnp_data_->object_points.size() - 4);
        if (pnp_data_->image_points.size()) {
          onPublisherButton();
          updateLaser2Image();
          updateLaser2Map();
          drawImages();
        }
      }
    }
    unlock();
  }

  void ImageView::drawImages() {
    for (size_t i = 0; i < image_properties_.laser2image_points_colored_.size(); ++i) {
      cv::circle(image_properties_.conversion_mat_,
                 image_properties_.laser2image_points_colored_[i].first,
                 2,
                 image_properties_.laser2image_points_colored_[i].second,
                 2);
    }

    if (image_properties_.conversion_mat_.rows > 0 && image_properties_.conversion_mat_.cols > 0) {
      QImage image(image_properties_.conversion_mat_.data,
                   image_properties_.conversion_mat_.cols,
                   image_properties_.conversion_mat_.rows,
                   image_properties_.conversion_mat_.step[0],
                   QImage::Format_RGB888);
      ui_.image_frame->setImage(image);
      ui_.image_frame->setInnerFrameFixedSize(image.size() * image_scale_factor_);
    }
  }

  void ImageView::shutdownPlugin() {
    subscriber_.shutdown();
  }

  void ImageView::onFreezeImageBoxToggle(bool val) {
    freeze_image_ = val;
  }

  void ImageView::onFreezeLaserBoxToggle(bool val) {
    freeze_laser_scan_ = val;
  }

  void ImageView::onLaserScanBoxToggle(bool val) {
    use_laser_scan_range_ = val;
  }

  void ImageView::onLeftSliderValChanged(int val) {
    lock();
    if (laser_properties_.has_laser_measurement()) {
      double angle_min = laser_properties_.measurement_laser_->getLaser().angle_min;
      double angle_max = laser_properties_.measurement_laser_->getLaser().angle_max;

      sliders_.leftSplitAngle_ = ((static_cast<double>(val) / static_cast<double>(ui_.sliderLeftLaser->maximum())) *
                                  (angle_max - angle_min)) + angle_min;

      updateLaser2Map();
      updateLaser2Image();
      drawImages();
    }
    unlock();

    ui_.sliderLeftLaserD->setValue(ui_.sliderLeftLaserD->maximum() * 0.5);
  }

  void ImageView::onLeftDistanceSliderValChanged(int val) {
    lock();

    double p = static_cast<double>(val) / static_cast<double>(ui_.sliderLeftLaserD->maximum());
    sliders_.distance_adjustment_left_ = (p - 0.5) * sliders_.distance_step_;

    updateLaser2Map();
    updateLaser2Image();
    drawImages();

    unlock();
  }

  void ImageView::onRightSliderValChanged(int val) {
    lock();

    if (laser_properties_.has_laser_measurement()) {
      double angle_min = 0, angle_max = 1;
      {
        angle_min = laser_properties_.measurement_laser_->getLaser().angle_min;
        angle_max = laser_properties_.measurement_laser_->getLaser().angle_max;
      }

      sliders_.rightSplitAngle_ =
          ((static_cast<double>(val) / static_cast<double>(ui_.sliderRightLaser->maximum())) *
           (angle_max - angle_min)) + angle_min;

      updateLaser2Map();
      updateLaser2Image();
      drawImages();
    }
    unlock();

    ui_.sliderRightLaserD->setValue(ui_.sliderRightLaserD->maximum() * 0.5);
  }

  void ImageView::onRightDistanceSliderValChanged(int val) {
    lock();
    double p = static_cast<double>(val) / static_cast<double>(ui_.sliderLeftLaserD->maximum());
    sliders_.distance_adjustment_right_ = (p - 0.5) * sliders_.distance_step_;

    updateLaser2Map();
    updateLaser2Image();
    drawImages();

    unlock();
  }

  void ImageView::onZoomIn() {
    image_scale_factor_ += 0.1;
    scaleImage(image_scale_factor_);
  }

  void ImageView::onZoomOut() {
    image_scale_factor_ -= 0.1;
    scaleImage(image_scale_factor_);
  }

  void ImageView::scaleImage(double factor) {
    if (ui_.image_frame->getImage().isNull()) {
      return;
    }
    ui_.image_frame->setInnerFrameFixedSize(ui_.image_frame->getImage().size() * factor);
  }

  void ImageView::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const {
    QString topic_img = ui_.topics_combo_box->currentText();
    QString topic_laser = ui_.laser_topic_combobox->currentText();
    QString pub_topic = ui_.pub_topic_textfield->text();
    instance_settings.setValue("topic_image", topic_img);
    plugin_settings.setValue("topic_image", topic_img);

    instance_settings.setValue("topic_laser", topic_laser);
    plugin_settings.setValue("topic_laser", topic_laser);

    instance_settings.setValue("base_link_topic", base_link_topic_);
    plugin_settings.setValue("base_link_topic", base_link_topic_);

    instance_settings.setValue("pub_topic", publisher_topic_);
    plugin_settings.setValue("pub_topic", publisher_topic_);
  }

  void ImageView::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                  const qt_gui_cpp::Settings &instance_settings) {
    QString topic = instance_settings.value("topic_image", "").toString();
    if (!arg_topic_name.isEmpty()) {
      arg_topic_name = "";
    } else {
      selectTopic(topic);
    }
    QString topic_laser = instance_settings.value("topic_laser", "").toString();
    if (!topic_laser.isEmpty()) {
      selectLaserTopic(topic_laser);
    }

    publisher_topic_ = instance_settings.value("pub_topic", "").toString();
    base_link_topic_ = instance_settings.value("base_link_topic", "").toString();

    ui_.pub_topic_textfield->setText(publisher_topic_);
    ui_.base_link_textfield->setText(base_link_topic_);
  }

  void ImageView::updateLaserTopicList() {
    QSet<QString> message_types;
    QSet<QString> message_sub_types;
    QList<QString> transports_unused;

    message_types.insert("sensor_msgs/LaserScan");

    QString selected = ui_.laser_topic_combobox->currentText();

    QList<QString> topics = getTopics(message_types, message_sub_types, transports_unused).values();
    topics.append("");
    qSort(topics);

    ui_.laser_topic_combobox->clear();

    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); ++it) {
      QString label(*it);
      label.replace(" ", "/");
      ui_.laser_topic_combobox->addItem(label, QVariant(*it));
    }

    selectLaserTopic(selected);
  }

  void ImageView::updateTopicList() {
    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");
    QSet<QString> message_sub_types;
    message_sub_types.insert("sensor_msgs/CompressedImage");

    // get declared transports
    QList<QString> transports;
    image_transport::ImageTransport it(getNodeHandle());
    std::vector<std::string> declared = it.getDeclaredTransports();
    for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++) {
      //qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
      QString transport = it->c_str();

      // strip prefix from transport name
      QString prefix = "image_transport/";
      if (transport.startsWith(prefix)) {
        transport = transport.mid(prefix.length());
      }
      transports.append(transport);
    }

    QString selected = ui_.topics_combo_box->currentText();

    // fill combo box
    QList<QString> topics = getTopics(message_types, message_sub_types, transports).values();
    topics.append("");
    qSort(topics);
    ui_.topics_combo_box->clear();
    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++) {
      QString label(*it);
      label.replace(" ", "/");
      ui_.topics_combo_box->addItem(label, QVariant(*it));
    }

    // restore previous selection
    selectTopic(selected);
  }

  QList<QString> ImageView::getTopicList(const QSet<QString> &message_types, const QList<QString> &transports) {
    QSet<QString> message_sub_types;
    return getTopics(message_types, message_sub_types, transports).values();
  }

  QSet<QString> ImageView::getTopics(const QSet<QString> &message_types, const QSet<QString> &message_sub_types,
                                     const QList<QString> &transports) {
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    QSet<QString> all_topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++) {
      all_topics.insert(it->name.c_str());
    }

    QSet<QString> topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++) {
      if (message_types.contains(it->datatype.c_str())) {
        QString topic = it->name.c_str();

        // add raw topic
        topics.insert(topic);

        // add transport specific sub-topics
        for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++) {
          if (all_topics.contains(topic + "/" + *jt)) {
            QString sub = topic + " " + *jt;
            topics.insert(sub);
          }
        }
      }
      if (message_sub_types.contains(it->datatype.c_str())) {
        QString topic = it->name.c_str();
        int index = topic.lastIndexOf("/");
        if (index != -1) {
          topic.replace(index, 1, " ");
          topics.insert(topic);
          //qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
        }
      }
    }
    return topics;
  }

  void ImageView::selectLaserTopic(const QString &topic) {
    int index = ui_.laser_topic_combobox->findText(topic);
    if (index == -1) {
      // add topic name to list if not yet in
      QString label(topic);
      label.replace(" ", "/");
      ui_.laser_topic_combobox->addItem(label, QVariant(topic));
      index = ui_.laser_topic_combobox->findText(topic);
    }
    ui_.laser_topic_combobox->setCurrentIndex(index);
  }

  void ImageView::selectTopic(const QString &topic) {
    int index = ui_.topics_combo_box->findText(topic);
    if (index == -1) {
      // add topic name to list if not yet in
      QString label(topic);
      label.replace(" ", "/");
      ui_.topics_combo_box->addItem(label, QVariant(topic));
      index = ui_.topics_combo_box->findText(topic);
    }
    ui_.topics_combo_box->setCurrentIndex(index);
  }

  void ImageView::onTopicChanged(int index) {
    subscriber_.shutdown();

    // reset image on topic change
    ui_.image_frame->setImage(QImage());

    QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";

    std::string image_topic = topic.toStdString();
    std::string topic_camera_info;

    if (!topic.isEmpty()) {

      for (size_t i = image_topic.length() - 1; i >= 0; --i) {
        if (image_topic[i] == '/') {
          topic_camera_info.resize(i + 1);
          std::copy(image_topic.begin(), image_topic.begin() + i + 1,
                    topic_camera_info.begin());
          break;
        }
      }

      topic_camera_info += "camera_info";

      image_transport::ImageTransport it(getNodeHandle());
      image_transport::TransportHints hints(transport.toStdString());

      try {
        subscriber_ = it.subscribe(topic.toStdString(), 1, &ImageView::callbackImage, this, hints);
        //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
      } catch (image_transport::TransportLoadException &e) {
        QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
      }

      sub_camera_info_ = getNodeHandle().subscribe(topic_camera_info, 1, &ImageView::callbackCameraInfo, this);
      //@ToDo: remove hardcoded
      sub_laser_ = getNodeHandle().subscribe(std::string("/r0/laser0/scan"), 1, &ImageView::callbackLaser, this);
    }

    onMousePublish(true);
  }

  void ImageView::onMousePublish(bool checked) {
    //std::string topicName;
    //if ( pub_topic_custom_ ) {
    //  topicName = ui_.publish_click_location_topic_line_edit->text().toStdString();
    //} else {
    //  if ( !subscriber_.getTopic().empty()) {
    //    topicName = subscriber_.getTopic() + "_mouse_left";
    //  } else {
    //    topicName = "mouse_left";
    //  }
    //  ui_.publish_click_location_topic_line_edit->setText( QString::fromStdString( topicName ));
    //}

    //pub_fiducial_detection_ = getNodeHandle().advertise<marker_msgs::FiducialDetection>( "/fiducials", 1000 );
  }

  void ImageView::onMouseLeft(int x, int y) {
    if (!ui_.image_frame->getImage().isNull()) {
      geometry_msgs::Point clickCanvasLocation;
      // Publish click location in pixel coordinates
      clickCanvasLocation.x = round(
          (double) x / (double) ui_.image_frame->width() * (double) ui_.image_frame->getImage().width());
      clickCanvasLocation.y = round(
          (double) y / (double) ui_.image_frame->height() * (double) ui_.image_frame->getImage().height());
      clickCanvasLocation.z = 0;

      lock();
      image_properties_.clickedPoints_.push_back(cv::Point2d(clickCanvasLocation.x, clickCanvasLocation.y));
      cv::circle(image_properties_.conversion_mat_,
                 image_properties_.clickedPoints_.back(),
                 2,
                 cv::Scalar(0, 255, 0),
                 2);
      QImage image(image_properties_.conversion_mat_.data,
                   image_properties_.conversion_mat_.cols,
                   image_properties_.conversion_mat_.rows,
                   image_properties_.conversion_mat_.step[0],
                   QImage::Format_RGB888);

      ui_.image_frame->setImage(image);

      unlock();
    }
  }

  void ImageView::onPubTopicChanged() {
    publisher_topic_ = ui_.pub_topic_textfield->text();
    pub_result_ = getNodeHandle().advertise<geometry_msgs::TransformStamped>(publisher_topic_.toStdString(), 1000);
  }

  void ImageView::onBaseTFTopicChanged() {
    if (!ui_.base_link_textfield->text().isEmpty()) {
      base_link_topic_ = ui_.base_link_textfield->text();
      if (base_link_topic_[0] == '/') {
        base_link_topic_ = base_link_topic_.right(base_link_topic_.size() - 1);
      }
    }
  }

  void ImageView::updateLaser2Map() {

    if (laser_properties_.has_laser_measurement() && laser_properties_.is_figure_initialized()) {

      laser_properties_.figure_local_->clear();
      laser_properties_.figure_local_->init(700, 700, -1, laser_properties_.measurement_laser_->max_reading_ + 1, -5, 5,
                                            M_PI / 2.0, 1, 1);

      bool first_hit = false;

      sliders_.currentLeftRange = 100;
      sliders_.currentRightRange = 100;

      for (std::vector<tuw::Contour::Beam>::iterator it_l = laser_properties_.measurement_laser_->begin();
           it_l != laser_properties_.measurement_laser_->end();
           ++it_l) {
        //double colorscale = (it_l->angle - min_angle) / (max_angle - min_angle);
        if (it_l->angle < sliders_.leftSplitAngle_ && it_l->angle > sliders_.rightSplitAngle_) {
          if (!first_hit) {
            first_hit = true;
            sliders_.currentRightRange = it_l->range;
          } else {
            sliders_.currentLeftRange = it_l->range;
          }
          laser_properties_.figure_local_->circle(it_l->end_point, 2, laser_properties_.figure_local_->green);
          it_l->set_valid(true);
        } else {
          laser_properties_.figure_local_->circle(it_l->end_point, 2, laser_properties_.figure_local_->red);
          it_l->set_valid(false);
        }
      }

      sliders_.currentLeftRange += sliders_.distance_adjustment_left_;
      sliders_.currentRightRange += sliders_.distance_adjustment_right_;

      double coss = cos(sliders_.leftSplitAngle_) * sliders_.currentLeftRange;
      double sinn = sin(sliders_.leftSplitAngle_) * sliders_.currentLeftRange;
      auto pleft = tuw::Point2D(coss, sinn);
      coss = cos(sliders_.rightSplitAngle_) * sliders_.currentRightRange;
      sinn = sin(sliders_.rightSplitAngle_) * sliders_.currentRightRange;
      auto pright = tuw::Point2D(coss, sinn);

      laser_properties_.figure_local_->line(tuw::Point2D(0, 0), pleft, laser_properties_.figure_local_->blue);
      laser_properties_.figure_local_->circle(pleft, 2, laser_properties_.figure_local_->blue, 2);
      laser_properties_.figure_local_->line(tuw::Point2D(0, 0), pright, laser_properties_.figure_local_->blue);
      laser_properties_.figure_local_->circle(pright, 2, laser_properties_.figure_local_->blue, 2);

      cv::Mat view = laser_properties_.figure_local_->view();
      //cv::imshow( "view", view );
      QImage image(view.data, view.cols, view.rows, view.step[0],
                   QImage::Format_RGB888);
      ui_.laser_frame->setImage(image);
      ui_.laser_frame->setInnerFrameFixedSize(image.size() * laser_scale_factor_);
      ui_.laser_frame->setEnabled(true);
      ui_.laser_frame->setVisible(true);

      //std::cout << std::to_string( ui_.laser_frame->size().width()) << ", "
      //          << std::to_string( ui_.laser_frame->size().height()) << std::endl;
    }
  }

  void ImageView::onRefinePressed() {
    ui_.freeze_laser_checkbox->setChecked(false);
    ui_.freeze_image_checkbox->setChecked(false);
    ui_.laser_scan_checkbox->setChecked(false);
    lock();
    image_properties_.clickedPoints_.clear();

    updateLaser2Image();
    drawImages();
    unlock();
  }

  bool ImageView::updateLaser2Image() {

    if (image_properties_.has_image_measurement() &&
        image_properties_.measurement_image_->getCameraModel() &&
        laser_properties_.has_laser_measurement() &&
        laser_properties_.is_figure_initialized()) {

      image_properties_.laser2image_points_colored_.clear();
      Eigen::Matrix4d T_WC;
      Eigen::Matrix4d T_WL;

      T_WC = image_properties_.measurement_image_->getTfWorldSensor();
      T_WL = laser_properties_.measurement_laser_->getTfWorldSensor();

      const Eigen::Matrix<double, 4, 4> T_CL = T_WC.inverse() * T_WL;

      bool isID = T_CL(0, 1) == 0 && T_CL(0, 2) == 0 && T_CL(0, 1) == 0 && T_CL(0, 2) == 0;
      isID &= T_CL(1, 0) == 0 && T_CL(1, 2) == 0 && T_CL(1, 3) == 0;
      isID &= T_CL(2, 0) == 0 && T_CL(2, 1) == 0 && T_CL(2, 3) == 0;
      isID &= T_CL(3, 0) == 0 && T_CL(3, 1) == 0 && T_CL(3, 2) == 0;

      //std::cout << "T_CL " << T_CL << std::endl;
      image_properties_.laser2image_points_colored_.resize(laser_properties_.measurement_laser_->size());
      std::size_t i = 0;
      for (auto beam_it = laser_properties_.measurement_laser_->begin();
           beam_it != laser_properties_.measurement_laser_->end();
           ++beam_it, ++i) {
        Eigen::Vector4d laser_in_image =
            T_CL * Eigen::Vector4d(beam_it->end_point.x(), beam_it->end_point.y(), 0, 1);
        laser_in_image = laser_in_image / laser_in_image[3];

        if (isID) {
          std::swap(laser_in_image[1], laser_in_image[2]); //y is z
        }
        const cv::Point3d pnt3d = cv::Point3d(laser_in_image[0], laser_in_image[1], laser_in_image[2]);
        cv::Scalar color = laser_properties_.figure_local_->magenta;
        if (beam_it->is_valid()) {
          color = laser_properties_.figure_local_->green;
        }
        image_properties_.laser2image_points_colored_[i] = std::make_pair(
            image_properties_.measurement_image_->getCameraModel()->project3dToPixel(pnt3d),
            color);
      }
      return true;
    }
    return false;
  }

  void ImageView::setIdentity(geometry_msgs::TransformStampedPtr tf) {
    auto &q = tf->transform.rotation;
    auto &t = tf->transform.translation;

    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;

    t.x = 0.0;
    t.y = 0.0;
    t.z = 0.0;
  }

  void ImageView::callbackLaser(const sensor_msgs::LaserScan &_laser) {
    //@ToDo: here
    lock();
    geometry_msgs::TransformStampedPtr tf_ptr;
    tf_ptr.reset(new geometry_msgs::TransformStamped());
    setIdentity(tf_ptr);

    if (getStaticTF(base_link_topic_.toStdString(), _laser.header.frame_id.c_str(), tf_ptr, false)) {
      if (laser_properties_.has_laser_measurement()) {
        laser_properties_.measurement_laser_->setTfWorldSensor(tf_ptr);
      }
    }

    if (!freeze_laser_scan_) {
      image_properties_.laser2image_points_colored_.clear();

      if (!laser_properties_.has_laser_measurement()) {
        laser_properties_.measurement_laser_.reset(new tuw::LaserMeasurement(tf_ptr));
      }
      laser_properties_.measurement_laser_->initFromScan(_laser);
    }

    //Auto check if measurement is there in method
    updateLaser2Map();
    updateLaser2Image();
    drawImages();

    unlock();
  }

  void ImageView::callbackCameraInfo(const sensor_msgs::CameraInfo::ConstPtr &_msg) {
    //std::cout << "callbackCamerainfo" << std::endl;

    lock();
    camera_model_.reset(new image_geometry::PinholeCameraModel());
    camera_model_->fromCameraInfo(_msg);
    if (image_properties_.has_image_measurement()) {
      image_properties_.measurement_image_->setCameraModel(camera_model_);
    }
    unlock();
  }

  void ImageView::callbackImage(const sensor_msgs::Image::ConstPtr &msg) {
    lock();
    geometry_msgs::TransformStampedPtr tf;
    tf.reset(new geometry_msgs::TransformStamped());
    setIdentity(tf);
    //if ( getStaticTF( "/r0/base_link", msg->header.frame_id.c_str(), tf, false )) {
    //  std::cout << "success" << std::endl;
    //}

    if (!freeze_image_) {

      try {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        image_properties_.conversion_mat_ = cv_ptr->image;

        if (image_properties_.has_image_measurement()) {
          tf = image_properties_.measurement_image_->getStampedTf();
        }

        image_properties_.measurement_image_.reset(new tuw::ImageMeasurement(cv_ptr, tf));

        if (camera_model_) {
          image_properties_.measurement_image_->setCameraModel(camera_model_);
        }
      }
      catch (cv_bridge::Exception &e) {
        try {
          // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
          if (msg->encoding == "CV_8UC3") {
            // assuming it is rgb
            image_properties_.conversion_mat_ = cv_ptr->image;
          } else if (msg->encoding == "8UC1") {
            // convert gray to rgb
            cv::cvtColor(cv_ptr->image, image_properties_.conversion_mat_, CV_GRAY2RGB);
          } else {
            qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)",
                     msg->encoding.c_str(),
                     e.what());
            ui_.image_frame->setImage(QImage());
            return;
          }
        }
        catch (cv_bridge::Exception &e) {
          qWarning(
              "ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)",
              msg->encoding.c_str(), e.what());
          ui_.image_frame->setImage(QImage());
          return;
        }
      }
    }

    updateLaser2Image();
    updateLaser2Map();
    drawImages();
    unlock();
  }

  void ImageView::onPublisherButton() {

    lock();
    if (image_properties_.clickedPoints_.size() == 4 &&
        use_laser_scan_range_ &&
        laser_properties_.has_laser_measurement()) {

      std::vector<cv::Point3f> object_points;
      Eigen::Vector4d leftMostPnt;
      Eigen::Vector4d rightMostPnt;
      Eigen::Matrix4d T_WL;
      {
        std::vector<tuw::Contour::Beam> laser_beams(laser_properties_.measurement_laser_->size());
        auto end_it = std::copy_if(laser_properties_.measurement_laser_->begin(),
                                   laser_properties_.measurement_laser_->end(), laser_beams.begin(),
                                   [](const tuw::Contour::Beam &b) {
                                     if (b.is_valid()) {
                                       return true;
                                     }
                                     return false;
                                   });
        laser_beams.resize(std::distance(laser_beams.begin(), end_it));

        if (fabs(sliders_.distance_adjustment_left_) < std::numeric_limits<double>::epsilon()) {
          leftMostPnt = Eigen::Vector4d(laser_beams.back().end_point.x(), laser_beams.back().end_point.y(), 0, 1);
        } else {
          auto &l_meas = laser_beams.back();
          double c_mod = cos(l_meas.angle) * (l_meas.range + sliders_.distance_adjustment_right_);
          double s_mod = sin(l_meas.angle) * (l_meas.range + sliders_.distance_adjustment_right_);
          leftMostPnt = Eigen::Vector4d(c_mod, s_mod, 0, 1);
        }

        if (fabs(sliders_.distance_adjustment_right_) < std::numeric_limits<double>::epsilon()) {
          rightMostPnt = Eigen::Vector4d(laser_beams.front().end_point.x(), laser_beams.front().end_point.y(), 0, 1);
        } else {
          auto &r_meas = laser_beams.front();
          double c_mod = cos(r_meas.angle) * (r_meas.range + sliders_.distance_adjustment_right_);
          double s_mod = sin(r_meas.angle) * (r_meas.range + sliders_.distance_adjustment_right_);
          rightMostPnt = Eigen::Vector4d(c_mod, s_mod, 0, 1);
        }
        T_WL = laser_properties_.measurement_laser_->getTfWorldSensor();
      }

      Eigen::Vector4d left_beam_base = T_WL * leftMostPnt;
      leftMostPnt = leftMostPnt / leftMostPnt[3];
      Eigen::Vector4d right_beam_base = T_WL * rightMostPnt;
      rightMostPnt = rightMostPnt / rightMostPnt[3];

      size_t point_count = 0;
      for (const cv::Point2d &pt : image_properties_.clickedPoints_) {

        cv::Point3d object_point;
        if (point_count == 0) {
          object_point.x = leftMostPnt.x();
          object_point.y = leftMostPnt.y();
          object_point.z = 0;
        } else if (point_count == 1) {
          object_point.x = leftMostPnt.x();
          object_point.y = leftMostPnt.y();
          object_point.z = 2.0f;
        } else if (point_count == 2) {
          object_point.x = rightMostPnt.x();
          object_point.y = rightMostPnt.y();
          object_point.z = 2.0f;
        } else if (point_count == 3) {
          object_point.x = rightMostPnt.x();
          object_point.y = rightMostPnt.y();
          object_point.z = 0.0f;
        }

        object_points.push_back(object_point);
        point_count++;

      }

      if (!pnp_data_) {
        pnp_data_.reset(new PnPData());
        pnp_data_->K = image_properties_.measurement_image_->getCameraModel()->intrinsicMatrix();
        pnp_data_->D = cv::Mat::zeros(cv::Size(1, 5), CV_64F);
      }

      cv::Mat rv, tv;

      pnp_data_->image_points.insert(std::begin(pnp_data_->image_points),
                                     std::begin(image_properties_.clickedPoints_),
                                     std::end(image_properties_.clickedPoints_));
      pnp_data_->object_points.insert(std::begin(pnp_data_->object_points), std::begin(object_points),
                                      std::end(object_points));

      cv::solvePnP(pnp_data_->object_points,
                   pnp_data_->image_points,
                   pnp_data_->K,
                   pnp_data_->D, rv, tv);

      pnp_data_->T_CW = cv::Mat::eye(4, 4, CV_64FC1);
      cv::Mat R33 = cv::Mat(pnp_data_->T_CW, cv::Rect(0, 0, 3, 3));
      cv::Rodrigues(rv, R33);

      for (int i = 0; i < 3; ++i) {
        pnp_data_->T_CW.row(i).col(3) = tv.at<double>(i);
      }

      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> T_CW(
          reinterpret_cast<double *>(pnp_data_->T_CW.data));
      Eigen::Matrix4d T_WC = T_CW.inverse();
      {
        image_properties_.measurement_image_->setTfWorldSensor(T_WC, true);
      }

      std::cout << "T_WC " << std::endl << T_WC << std::endl;

      updateLaser2Image();
      updateLaser2Map();
      drawImages();

      image_properties_.clickedPoints_.clear();
    }

    unlock();
  }

  bool ImageView::getStaticTF(const std::string &world_frame, const std::string &source_frame,
                              geometry_msgs::TransformStampedPtr &_tf, bool debug) {

    std::string target_frame_id = source_frame;
    std::string source_frame_id = world_frame;
    std::string key = target_frame_id + "->" + source_frame_id;

    if (!tfMap_[key]) {
      try {
        geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
            source_frame_id, target_frame_id, ros::Time(0));

        _tf.reset(new geometry_msgs::TransformStamped(stamped_tf));

        tfMap_[key] = _tf;
      } catch (tf2::TransformException &ex) {

        ROS_INFO("getStaticTF");
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;

      }
    } else {
      _tf = tfMap_[key];
    }

    if (debug) {
      std::cout << key << std::endl;
      const auto t = _tf->transform.translation;
      const auto q = _tf->transform.rotation;
      std::cout << "tf get o: " << t.x << ", " << t.y << ", "
                << t.z
                << std::endl;
      std::cout << "tf get r: " << q.x << ", " << q.y << ", "
                << q.z << ", " << q.w << std::endl;
    }

    return true;
  }
}

PLUGINLIB_EXPORT_CLASS(tuw_extrinsic_camera::ImageView, rqt_gui_cpp::Plugin)
