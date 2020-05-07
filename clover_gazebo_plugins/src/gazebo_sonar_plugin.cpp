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

#include "gazebo_sonar_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(SonarPlugin)

/////////////////////////////////////////////////
SonarPlugin::SonarPlugin()
{
}

/////////////////////////////////////////////////
SonarPlugin::~SonarPlugin()
{
  newScansConnection_->~Connection();
  newScansConnection_.reset();
  parentSensor_.reset();
  world_.reset();
}

/////////////////////////////////////////////////
void SonarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  //  Get then name of the parent sensor
  parentSensor_ = std::dynamic_pointer_cast<sensors::SonarSensor>(_parent);

  if (!parentSensor_)
    gzthrow("SonarPlugin requires a Sonar Sensor as its parent");

  world_ = physics::get_world(parentSensor_->WorldName());

  parentSensor_->SetActive(false);
  newScansConnection_ = parentSensor_->ConnectUpdated(boost::bind(&SonarPlugin::OnNewScans, this));
  parentSensor_->SetActive(true);

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_sonar_plugin] Please specify a robotNamespace.\n";

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

  // get sonar topic name
  if(_sdf->HasElement("topic")) {
    sonar_topic_ = parentSensor_->Topic();
  } else {
    // if not set by parameter, get the topic name from the model name
    sonar_topic_ = parentSensorModelName;
    gzwarn << "[gazebo_sonar_plugin]: " + names_splitted.front() + "::" + names_splitted.rbegin()[1] +
      " using sonar topic \"" << parentSensorModelName << "\"\n";
  }

  // Calculate parent sensor rotation WRT `base_link`
  const ignition::math::Quaterniond q_ls = parentSensor_->Pose().Rot();

  // set the orientation
  orientation_.set_x(q_ls.X());
  orientation_.set_y(q_ls.Y());
  orientation_.set_z(q_ls.Z());
  orientation_.set_w(q_ls.W());

  // start sonar topic publishing
  sonar_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Range>("~/" + names_splitted[0] + "/link/" + sonar_topic_, 10);
}

void SonarPlugin::OnNewScans()
{
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif

  sonar_message_.set_time_usec(now.Double() * 1e6);
  sonar_message_.set_min_distance(parentSensor_->RangeMin());
  sonar_message_.set_max_distance(parentSensor_->RangeMax());
  sonar_message_.set_current_distance(parentSensor_->Range());

  sonar_message_.set_h_fov(2.0f * atan(parentSensor_->Radius() / parentSensor_->RangeMax()));
  sonar_message_.set_v_fov(2.0f * atan(parentSensor_->Radius() / parentSensor_->RangeMax()));
  sonar_message_.set_allocated_orientation(new gazebo::msgs::Quaternion(orientation_));

  sonar_pub_->Publish(sonar_message_);
}
