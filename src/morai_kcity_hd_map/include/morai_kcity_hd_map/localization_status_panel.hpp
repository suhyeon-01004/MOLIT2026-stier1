#pragma once

#include <QString>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>

class QPlainTextEdit;

namespace morai_kcity_hd_map {

class LocalizationStatusPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit LocalizationStatusPanel(QWidget* parent = nullptr);

 Q_SIGNALS:
  void statusReceived(const QString& text);

 private Q_SLOTS:
  void appendStatus(const QString& text);

 private:
  void handleStatus(const std_msgs::String::ConstPtr& message);

  ros::NodeHandle node_;
  ros::Subscriber subscriber_;
  QPlainTextEdit* log_;
};

}  // namespace morai_kcity_hd_map
