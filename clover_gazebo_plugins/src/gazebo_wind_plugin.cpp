/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_wind_plugin.h"
#include "common.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  update_connection_->~Connection();
}

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, "/" + namespace_ + wind_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind params from SDF.
  getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_, wind_force_mean_);
  getSdfParam<double>(_sdf, "windForceMax", wind_force_max_, wind_force_max_);
  getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_, wind_force_variance_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
  getSdfParam<double>(_sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_, wind_gust_force_mean_);
  getSdfParam<double>(_sdf, "windGustForceMax", wind_gust_force_max_, wind_gust_force_max_);
  getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_, wind_gust_force_variance_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
  getSdfParam<double>(_sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);

  wind_direction_mean_.Normalize();
  wind_gust_direction_mean_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
  // Set random wind force mean and standard deviation
  wind_force_distribution_.param(std::normal_distribution<double>::param_type(wind_force_mean_, sqrt(wind_force_variance_)));
  // Set random wind direction mean and standard deviation
  wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
  // Set random wind gust force mean and standard deviation
  wind_gust_force_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_force_mean_, sqrt(wind_gust_force_variance_)));
  // Set random wind gust direction mean and standard deviation
  wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>(wind_pub_topic_, 1);
}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif

  // Calculate the wind force.
  // Get normal distribution wind strength
  double wind_strength = wind_force_distribution_(wind_force_generator_);
  wind_strength = (wind_strength > wind_force_max_) ? wind_force_max_ : wind_strength;
  // Get normal distribution wind direction
  ignition::math::Vector3d wind_direction;
  wind_direction.X() = wind_direction_distribution_X_(wind_direction_generator_);
  wind_direction.Y() = wind_direction_distribution_Y_(wind_direction_generator_);
  wind_direction.Z() = wind_direction_distribution_Z_(wind_direction_generator_);
  // Calculate total wind force
  ignition::math::Vector3d wind = wind_strength * wind_direction;
  // Apply a force from the constant wind to the link.
  link_->AddForceAtRelativePosition(wind, xyz_offset_);

  ignition::math::Vector3d wind_gust(0, 0, 0);
  // Calculate the wind gust force.
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    // Get normal distribution wind gust strength
    double wind_gust_strength = wind_gust_force_distribution_(wind_gust_force_generator_);
    wind_gust_strength = (wind_gust_strength > wind_gust_force_max_) ? wind_gust_force_max_ : wind_gust_strength;
    // Get normal distribution wind gust direction
    ignition::math::Vector3d wind_gust_direction;
    wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
    wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
    wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
    wind_gust = wind_gust_strength * wind_gust_direction;
    // Apply a force from the wind gust to the link.
    link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
  }

  gazebo::msgs::Vector3d* force = new gazebo::msgs::Vector3d();
  force->set_x(wind.X() + wind_gust.X());
  force->set_y(wind.Y() + wind_gust.Y());
  force->set_z(wind.Z() + wind_gust.Z());

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now.Double() * 1e6);
  wind_msg.set_allocated_force(force);

  wind_pub_->Publish(wind_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}
