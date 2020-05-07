/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "Int32.pb.h"
#include "gazebo_video_stream_widget.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(VideoStreamWidget)

/////////////////////////////////////////////////
VideoStreamWidget::VideoStreamWidget()
  : GUIPlugin()
  , mVideoON(true)
{
  this->counter = 0;

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  mButton = new QPushButton(tr("Video: OFF"));
  connect(mButton, SIGNAL(clicked()), this, SLOT(OnButton()));

  // Add the button to the frame's layout
  frameLayout->addWidget(mButton);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(100, 30);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->videoPub = this->node->Advertise<msgs::Int>("~/video_stream");
  videoPub->WaitForConnection();

  if(mVideoON)
    enable();
  else
    disable();

}

/////////////////////////////////////////////////
VideoStreamWidget::~VideoStreamWidget()
{
}

/////////////////////////////////////////////////
void VideoStreamWidget::OnButton()
{
  if(mVideoON) {
    mVideoON = false;
    disable();
  } else {
    mVideoON = true;
    enable();
  }

}

void VideoStreamWidget::enable()
{
  gzwarn << "Enable Video Streaming \n";

  msgs::Int request;
  request.set_data(1);
  videoPub->Publish(request);

  mButton->setText("Video: ON");
  mButton->setStyleSheet(QString("QPushButton {"
                                  "background-color: green; font-weight: bold"
                                  "}")
                             );

}

void VideoStreamWidget::disable()
{
  gzwarn << "Disable Video Streaming \n";

  msgs::Int request;
  request.set_data(0);
  videoPub->Publish(request);

  mButton->setText("Video: OFF");
  mButton->setStyleSheet(QString("QPushButton {"
                                  "background-color: red; font-weight: bold"
                                  "}")
                             );

}



