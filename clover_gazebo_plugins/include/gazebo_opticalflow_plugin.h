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
#ifndef _GAZEBO_OPTICAL_FLOW_PLUGIN_HH_
#define _GAZEBO_OPTICAL_FLOW_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#include "OpticalFlow.pb.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ignition/math.hh>

#include "flow_opencv.hpp"
#include "flow_px4.hpp"

#define DEFAULT_RATE 20
#define HAS_GYRO true

using namespace cv;
using namespace std;

namespace gazebo
{
  static const std::string kDefaultGyroTopic = "/px4flow/imu";

  class GAZEBO_VISIBLE OpticalFlowPlugin : public SensorPlugin
  {
    public:
      OpticalFlowPlugin();
      virtual ~OpticalFlowPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);
      void ImuCallback(ConstIMUPtr& _imu);

    protected:
      unsigned int width, height, depth;
      std::string format;
      sensors::CameraSensorPtr parentSensor;
      rendering::CameraPtr camera;
      physics::WorldPtr world;

    private:
      event::ConnectionPtr newFrameConnection;
      transport::PublisherPtr opticalFlow_pub_;
      transport::NodePtr node_handle_;
      transport::SubscriberPtr imuSub_;
      sensor_msgs::msgs::OpticalFlow opticalFlow_message;
      ignition::math::Vector3d opticalFlow_rate;
      std::string namespace_;
      std::string gyro_sub_topic_;
      OpticalFlowOpenCV *optical_flow_;
      // OpticalFlowPX4 *optical_flow_;

      float hfov_;
      int dt_us_;
      int output_rate_;
      float focal_length_;
      double first_frame_time_;
      uint32_t frame_time_us_;
      bool has_gyro_;
  };
}
#endif
