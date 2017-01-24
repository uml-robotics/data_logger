/*
 * Copyright (c) 2012, University of Massachusetts Lowell.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of University of Massachusetts Lowell. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Mikhail Medvedev */

#ifndef THIS_PACKAGE_NAME
#define THIS_PACKAGE_NAME "The name of this package, defined in CMakeLists.txt"
#endif
#include <ros/package.h>

#include "datalog_widget.h"

#include <QPushButton>
#include <QHBoxLayout>
#include <QIcon>
#include <QPixmap>
#include <QMessageBox>

#include <QTabWidget>
#include <QFormLayout>
#include <QLineEdit>
#include <QCheckBox>

#include <std_srvs/Empty.h>

namespace data_logger
{

DatalogWidget::DatalogWidget(QWidget * parent) :
        QWidget(parent),
        sub_logger_status_(
            nh_.subscribe("/data_logger/status", 1,
                          &DatalogWidget::loggerStatusCb, this)),
        is_recording_(false),
        is_highlighted_(false)
{

  QTabWidget* tabs_widget = new QTabWidget;

  // Logger Tab
  {
    QWidget * tab = new QWidget;
    tabs_widget->addTab(tab, "Logger");
    tab->setLayout(new QHBoxLayout);
    tab->layout()->setContentsMargins(QMargins());

    QPixmap pixmap(
        QString::fromStdString(
            ros::package::getPath(THIS_PACKAGE_NAME) + "/images/floppy.png"));
    QIcon * floppy_icon = new QIcon(pixmap);
    button_ = new QPushButton(*floppy_icon, "Start Logger");
    button_->setIconSize(pixmap.size());
    button_->setCheckable(true);
    button_->setSizePolicy(
        QSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum));
    connect(button_, SIGNAL(clicked()), this, SLOT(handleButton()));

    tab->layout()->addWidget(button_);
  }

  // Settings Tab
  {
    QWidget * tab = new QWidget;
    tabs_widget->addTab(tab, "Settings");
    QFormLayout * layout = new QFormLayout;
    tab->setLayout(layout);

    topic_editor_ = new QLineEdit;
    topic_editor_->setToolTip("Topic with std_msgs/String published to it.");
    connect(topic_editor_, SIGNAL( editingFinished()), this,
            SLOT(updateTopic()));
    layout->addRow("String topic:", topic_editor_);
    substring_editor_ = new QLineEdit;
    substring_editor_->setToolTip("Substring to match with received string.");
    connect(substring_editor_, SIGNAL( editingFinished()), this,
            SLOT(updateString()));
    layout->addRow("Trigger on substring:", substring_editor_);
    autostart_checkbox_ = new QCheckBox("Autostart");
    autostart_checkbox_->setToolTip(
        "If checked, automatically start logger when matching string"
        " has been received.");
    connect(autostart_checkbox_, SIGNAL( stateChanged(int)), this,
            SLOT(updateCheckbox(int)));
    layout->addWidget(autostart_checkbox_);
  }

  setLayout(new QHBoxLayout);
  layout()->setContentsMargins(QMargins());
  layout()->addWidget(tabs_widget);
}

void DatalogWidget::handleButton()
{
  std_srvs::Empty empty;
  updateButtonColor();
  if (button_->isChecked())
  {
    ros::service::call("/data_logger/start", empty);
  }
  else
  {
    button_->setVisible(false);
    // Pop up confirmation message before stopping the logger
    QMessageBox msgBox;
    msgBox.setParent(this);
    msgBox.setGeometry(button_->geometry());
    msgBox.setText("Confirm to stop logging.");
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    msgBox.show();
    msgBox.move((rect().width() - msgBox.size().width()) / 2,
                (rect().height() - msgBox.size().height()) / 2);
    if (msgBox.exec() == QMessageBox::Yes)
    {
      ros::service::call("/data_logger/stop", empty);
    }
    else
    {
      button_->setChecked(true);
    }
    button_->setVisible(true);
  }
}

void DatalogWidget::loggerStatusCb(const std_msgs::StringConstPtr& msg)
{
  logger_status_ = msg->data;
  if (logger_status_ == "started")
  {
    is_recording_ = true;
    button_->setText("Stop Logger");
  }
  else if (logger_status_ == "stopped" || logger_status_ == "")
  {
    is_recording_ = false;
    button_->setText("Start Logger");
  }
  button_->setChecked(is_recording_);
  updateButtonColor();
}

void DatalogWidget::updateButtonColor(){
  if (is_recording_){
    button_->setStyleSheet("QPushButton {background-color: red}");
  }else{
    button_->setStyleSheet("QPushButton {background-color: none}");
  }
}

void DatalogWidget::updateTopic()
{
  std::string new_topic = topic_editor_->text().toStdString();
  if (new_topic != trigger_topic_)
  {
    trigger_topic_ = new_topic;
    if (trigger_topic_ == "")
    {
      sub_trigger_topic_.shutdown();
    }
    else
    {
      sub_trigger_topic_ = nh_.subscribe(trigger_topic_, 1,
                                         &DatalogWidget::triggerTopicCb, this);
    }
    Q_EMIT configChanged();
  }
}

void DatalogWidget::triggerTopicCb(const std_msgs::StringConstPtr & msg)
{
  if (msg->data.find(substring_editor_->text().toStdString())
      != std::string::npos)
  {
    // Found matching substring in the trigger topic.
    // Would now flash the button if auto is not set,
    // would start the logger if auto is set.
    if (!button_->isChecked())
    {
      if (autostart_checkbox_->isChecked())
      {
        button_->setChecked(true);
        handleButton();
      }
      else
      {
        triggerButtonHighlight();
      }
    }
  }

}

void DatalogWidget::updateString()
{
  Q_EMIT configChanged();
}

void DatalogWidget::updateCheckbox()
{
  Q_EMIT configChanged();
}

void DatalogWidget::saveToConfig(const std::string& key_prefix,
                                 const boost::shared_ptr<rviz::Config>& config)
{
  config->set(key_prefix + "/TriggerTopic",
              topic_editor_->text().toStdString());
  config->set(key_prefix + "/SubstringMatch",
              substring_editor_->text().toStdString());
  config->set(key_prefix + "/Autostart", autostart_checkbox_->isChecked());
}

void DatalogWidget::loadFromConfig(
    const std::string& key_prefix,
    const boost::shared_ptr<rviz::Config>& config)
{
  std::string s;
  config->get(key_prefix + "/TriggerTopic", &s);
  topic_editor_->setText(QString::fromStdString(s));
  updateTopic();

  config->get(key_prefix + "/SubstringMatch", &s);
  substring_editor_->setText(QString::fromStdString(s));

  int is_checked;
  config->get(key_prefix + "/Autostart", &is_checked);
  autostart_checkbox_->setChecked(is_checked);
}

void DatalogWidget::triggerButtonHighlight()
{
  if (is_highlighted_)
  {
    button_->setStyleSheet("QPushButton {background-color: red}");
  }
  else
  {
    button_->setStyleSheet("QPushButton {background-color: none}");
  }
  is_highlighted_ = !is_highlighted_;
}

} /* namespace data_logger */
