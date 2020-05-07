/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/*
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "gazebo_lidar_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <common.h>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

/////////////////////////////////////////////////
RayPlugin::RayPlugin()
{
}

/////////////////////////////////////////////////
RayPlugin::~RayPlugin()
{
  newLaserScansConnection_->~Connection();
  newLaserScansConnection_.reset();
  parentSensor_.reset();
  world_->Reset();
}

/////////////////////////////////////////////////
void RayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  parentSensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!parentSensor_)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

  world_ = physics::get_world(parentSensor_->WorldName());

  newLaserScansConnection_ = parentSensor_->LaserShape()->ConnectNewLaserScans(
      boost::bind(&RayPlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  // get minimum distance
  if (_sdf->HasElement("min_distance")) {
    min_distance_ = _sdf->GetElement("min_distance")->Get<double>();
    if (min_distance_ < kSensorMinDistance) {
      min_distance_ = kSensorMinDistance;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default minimum distance: " << kDefaultMinDistance << "\n";
    min_distance_ = kDefaultMinDistance;
  }

  // get maximum distance
  if (_sdf->HasElement("max_distance")) {
    max_distance_ = _sdf->GetElement("max_distance")->Get<double>();
    if (max_distance_ > kSensorMaxDistance) {
      max_distance_ = kSensorMaxDistance;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default maximum distance: " << kDefaultMaxDistance << "\n";
    max_distance_ = kDefaultMaxDistance;
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Get the root model name
  const string scopedName = _parent->ParentName();
  vector<string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                            [](const string& name)
                            { return name.size() == 0; }), end(names_splitted));
  std::string rootModelName = names_splitted.front(); // The first element is the name of the root model

  // the second to the last name is the model name
  const std::string parentSensorModelName = names_splitted.rbegin()[1];

  // get lidar topic name
  if(_sdf->HasElement("topic")) {
    lidar_topic_ = parentSensor_->Topic();
  } else {
    // if not set by parameter, get the topic name from the model name
    lidar_topic_ = parentSensorModelName;
    gzwarn << "[gazebo_lidar_plugin]: " + names_splitted.front() + "::" + names_splitted.rbegin()[1] +
      " using lidar topic \"" << parentSensorModelName << "\"\n";
  }

  // Calculate parent sensor rotation WRT `base_link`
  const ignition::math::Quaterniond q_ls = parentSensor_->Pose().Rot();

  // Set the orientation
  orientation_.set_x(q_ls.X());
  orientation_.set_y(q_ls.Y());
  orientation_.set_z(q_ls.Z());
  orientation_.set_w(q_ls.W());

  // start lidar topic publishing
  lidar_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Range>("~/" + names_splitted[0] + "/link/" + lidar_topic_, 10);
}

/////////////////////////////////////////////////
void RayPlugin::OnNewLaserScans()
{
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif

  lidar_message_.set_time_usec(now.Double() * 1e6);
  lidar_message_.set_min_distance(min_distance_);
  lidar_message_.set_max_distance(max_distance_);

  double current_distance = parentSensor_->Range(0);

  // set distance to min/max if actual value is smaller/bigger
  if (current_distance < min_distance_ || std::isinf(current_distance)) {
    current_distance = min_distance_;
  } else if (current_distance > max_distance_) {
    current_distance = max_distance_;
  }

  lidar_message_.set_current_distance(current_distance);
  lidar_message_.set_h_fov(kDefaultFOV);
  lidar_message_.set_v_fov(kDefaultFOV);
  lidar_message_.set_allocated_orientation(new gazebo::msgs::Quaternion(orientation_));

  lidar_pub_->Publish(lidar_message_);
}
