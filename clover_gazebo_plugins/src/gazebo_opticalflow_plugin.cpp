/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_opticalflow_plugin.h"

#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(OpticalFlowPlugin)

/////////////////////////////////////////////////
OpticalFlowPlugin::OpticalFlowPlugin()
: SensorPlugin(), width(0), height(0), depth(0)
{

}

/////////////////////////////////////////////////
OpticalFlowPlugin::~OpticalFlowPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void OpticalFlowPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "OpticalFlowPlugin requires a CameraSensor.\n";
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
      gzmsg << "It is a depth camera sensor\n";
  }

  if (!this->parentSensor)
  {
    gzerr << "OpticalFlowPlugin not attached to a camera sensor\n";
    return;
  }

  this->world = physics::get_world(this->parentSensor->WorldName());

#if GAZEBO_MAJOR_VERSION >= 7
  this->camera = this->parentSensor->Camera();
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();
  hfov_ = float(this->camera->HFOV().Radian());
  first_frame_time_ = this->camera->LastRenderWallTime().Double();
  const string scopedName = _sensor->ParentName();
#else
  this->camera = this->parentSensor->GetCamera();
  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->depth = this->camera->GetImageDepth();
  this->format = this->camera->GetImageFormat();
  hfov_ = float(this->camera->GetHFOV().Radian());
  first_frame_time_ = this->camera->GetLastRenderWallTime().Double();
  const string scopedName = _sensor->GetParentName();
#endif

  focal_length_ = (this->width/2)/tan(hfov_/2);

  if (this->width != 64 || this->height != 64) {
    gzerr << "[gazebo_optical_flow_plugin] Incorrect image size, must by 64 x 64.\n";
  }

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_optical_flow_plugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("outputRate")) {
    output_rate_ = _sdf->GetElement("outputRate")->Get<int>();
  } else {
    output_rate_ = DEFAULT_RATE;
    gzwarn << "[gazebo_optical_flow_plugin] Using default output rate " << output_rate_ << ".";
  }

  if(_sdf->HasElement("hasGyro"))
    has_gyro_ = _sdf->GetElement("hasGyro")->Get<bool>();
  else
    has_gyro_ = HAS_GYRO;

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if(has_gyro_) {
    if(_sdf->HasElement("hasGyro"))
      gyro_sub_topic_ = _sdf->GetElement("gyroTopic")->Get<std::string>();
    else
      gyro_sub_topic_ = kDefaultGyroTopic;

    string topicName = "~/" + _sensor->ParentName() + gyro_sub_topic_;
    boost::replace_all(topicName, "::", "/");
    imuSub_ = node_handle_->Subscribe(topicName, &OpticalFlowPlugin::ImuCallback, this);
  }

  string topicName = "~/" + scopedName + "/opticalFlow";
  boost::replace_all(topicName, "::", "/");

  opticalFlow_pub_ = node_handle_->Advertise<sensor_msgs::msgs::OpticalFlow>(topicName, 10);

  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      boost::bind(&OpticalFlowPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

  this->parentSensor->SetActive(true);

  //init flow
  optical_flow_ = new OpticalFlowOpenCV(focal_length_, focal_length_, output_rate_);
  // _optical_flow = new OpticalFlowPX4(focal_length_, focal_length_, output_rate_, this->width);
}

/////////////////////////////////////////////////
void OpticalFlowPlugin::OnNewFrame(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int _depth,
                              const std::string &_format)
{

  //get data depending on gazebo version
  #if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera->ImageData(0);
    double frame_time = this->camera->LastRenderWallTime().Double();
  #else
    _image = this->camera->GetImageData(0);
    double frame_time = this->camera->GetLastRenderWallTime().Double();
  #endif

  frame_time_us_ = (frame_time - first_frame_time_) * 1e6; //since start

  float flow_x_ang = 0.0f;
  float flow_y_ang = 0.0f;
  //calculate angular flow
  int quality = optical_flow_->calcFlow((uchar*)_image, frame_time_us_, dt_us_, flow_x_ang, flow_y_ang);

  if (quality >= 0) { // calcFlow(...) returns -1 if data should not be published yet -> output_rate
    //prepare optical flow message
    // Get the current simulation time.
    #if GAZEBO_MAJOR_VERSION >= 9
      common::Time now = world->SimTime();
    #else
      common::Time now = world->GetSimTime();
    #endif

    opticalFlow_message.set_time_usec(now.Double() * 1e6);
    opticalFlow_message.set_sensor_id(2.0);
    opticalFlow_message.set_integration_time_us(quality ? dt_us_ : 0);
    opticalFlow_message.set_integrated_x(quality ? flow_x_ang : 0.0f);
    opticalFlow_message.set_integrated_y(quality ? flow_y_ang : 0.0f);
    if(has_gyro_) {
      opticalFlow_message.set_integrated_xgyro(opticalFlow_rate.X());
      opticalFlow_message.set_integrated_ygyro(opticalFlow_rate.Y());
      opticalFlow_message.set_integrated_zgyro(opticalFlow_rate.Z());
      //reset gyro integral
      opticalFlow_rate.Set();
    } else {
      //no gyro
      opticalFlow_message.set_integrated_xgyro(NAN);
      opticalFlow_message.set_integrated_ygyro(NAN);
      opticalFlow_message.set_integrated_zgyro(NAN);
    }
    opticalFlow_message.set_temperature(20.0f);
    opticalFlow_message.set_quality(quality);
    opticalFlow_message.set_time_delta_distance_us(0);
    opticalFlow_message.set_distance(0.0f); //get real values in gazebo_mavlink_interface.cpp
    //send message
    opticalFlow_pub_->Publish(opticalFlow_message);
  }
}

void OpticalFlowPlugin::ImuCallback(ConstIMUPtr& _imu) {
  //accumulate gyro measurements that are needed for the optical flow message
  #if GAZEBO_MAJOR_VERSION >= 9
    common::Time now = world->SimTime();
  #else
    common::Time now = world->GetSimTime();
  #endif

  uint32_t now_us = now.Double() * 1e6;
  ignition::math::Vector3d px4flow_gyro = ignition::math::Vector3d(_imu->angular_velocity().x(),
                                                                   _imu->angular_velocity().y(),
                                                                   _imu->angular_velocity().z());

  static uint32_t last_dt_us = now_us;
  uint32_t dt_us = now_us - last_dt_us;

  if (dt_us > 1000) {
    opticalFlow_rate += px4flow_gyro * (dt_us / 1000000.0f);
    last_dt_us = now_us;
  }
}
/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
