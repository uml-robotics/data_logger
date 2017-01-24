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

#ifndef DATALOG_WIDGET_H_
#define DATALOG_WIDGET_H_

#include <qwidget.h>

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>

class QPushButton;
class QLineEdit;
class QCheckBox;

namespace data_logger
{

/*
 *
 */
class DatalogWidget : public QWidget
{
Q_OBJECT
public:
  DatalogWidget(QWidget *parent = 0);

public Q_SLOTS:
  void handleButton();

protected:

  QPushButton * button_;

private:

  ros::NodeHandle nh_;
  ros::Subscriber sub_logger_status_;
  void loggerStatusCb(const std_msgs::StringConstPtr & msg);
  std::string logger_status_;

  bool is_recording_;
  void updateButtonColor();//< Change color based on is_recodring state
  bool is_highlighted_;
  void triggerButtonHighlight();

  // Settings
public:
  // How else can we save / load settings from withing a widget (not a rviz::Panel_)
  // without doing delegation in from parent panel?
  void saveToConfig(const std::string& key_prefix,
                    const boost::shared_ptr<rviz::Config>& config);
  void loadFromConfig(const std::string& key_prefix,
                      const boost::shared_ptr<rviz::Config>& config);
protected:
  ros::Subscriber sub_trigger_topic_;
  std::string trigger_topic_;
  void triggerTopicCb(const std_msgs::StringConstPtr & msg);
  QLineEdit * topic_editor_;
  QLineEdit * substring_editor_;
  QCheckBox * autostart_checkbox_;
protected Q_SLOTS:
  void updateTopic();
  void updateString();
  void updateCheckbox();

Q_SIGNALS:
  void configChanged();

};

} /* namespace data_logger */
#endif /* DATALOG_WIDGET_H_ */
