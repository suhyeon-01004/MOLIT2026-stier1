#include "morai_kcity_hd_map/localization_status_panel.hpp"

#include <QDateTime>
#include <QFontDatabase>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace morai_kcity_hd_map {

LocalizationStatusPanel::LocalizationStatusPanel(QWidget* parent)
    : rviz::Panel(parent), log_(new QPlainTextEdit(this)) {
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(1200);
  log_->setMinimumSize(330, 150);
  log_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  log_->setStyleSheet(
      "QPlainTextEdit { background: #171717; color: #e8e8e8; "
      "border: 1px solid #555; }");
  log_->appendPlainText("-- waiting for /localization/status_text --");

  auto* layout = new QVBoxLayout;
  layout->setContentsMargins(3, 3, 3, 3);
  layout->addWidget(log_);
  setLayout(layout);

  connect(this, &LocalizationStatusPanel::statusReceived,
          this, &LocalizationStatusPanel::appendStatus,
          Qt::QueuedConnection);
  subscriber_ = node_.subscribe(
      "/localization/status_text", 20,
      &LocalizationStatusPanel::handleStatus, this);
}

void LocalizationStatusPanel::handleStatus(
    const std_msgs::String::ConstPtr& message) {
  Q_EMIT statusReceived(QString::fromStdString(message->data));
}

void LocalizationStatusPanel::appendStatus(const QString& text) {
  const QString timestamp =
      QDateTime::currentDateTime().toString("HH:mm:ss.zzz");
  log_->appendPlainText(timestamp + " " + text);
  log_->verticalScrollBar()->setValue(log_->verticalScrollBar()->maximum());
}

}  // namespace morai_kcity_hd_map

PLUGINLIB_EXPORT_CLASS(morai_kcity_hd_map::LocalizationStatusPanel,
                       rviz::Panel)
