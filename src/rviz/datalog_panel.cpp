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

#include "datalog_panel.h"

#include "datalog_widget.h"
#include <QHBoxLayout>

namespace data_logger
{

DatalogPanel::DatalogPanel(QWidget *parent) :
        rviz::Panel(parent)
{
  datalog_widget_ = new DatalogWidget;
  connect(datalog_widget_, SIGNAL(configChanged() ), this,
          SLOT(emitConfigChanged()));

  auto layout = new QHBoxLayout;
  layout->setContentsMargins(QMargins());
  layout->addWidget(datalog_widget_);


  setLayout(layout);
}

void DatalogPanel::saveToConfig(const std::string& key_prefix,
                             const boost::shared_ptr<rviz::Config>& config)
{
  datalog_widget_->saveToConfig(key_prefix, config);
}

void DatalogPanel::loadFromConfig(const std::string& key_prefix,
                               const boost::shared_ptr<rviz::Config>& config)
{
  datalog_widget_->loadFromConfig(key_prefix, config);
}

} /* namespace data_logger */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS( data_logger, DataLogger, data_logger::DatalogPanel,
                        rviz::Panel)
