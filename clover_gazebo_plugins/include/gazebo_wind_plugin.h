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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Wind.pb.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

static constexpr double kDefaultWindForceMean = 0.0;
static constexpr double kDefaultWindForceMax = 100.0;
static constexpr double kDefaultWindForceVariance = 0.0;
static constexpr double kDefaultWindGustForceMean = 0.0;
static constexpr double kDefaultWindGustForceMax = 100.0;
static constexpr double kDefaultWindGustForceVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static const ignition::math::Vector3d kDefaultWindDirectionMean = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirectionMean = ignition::math::Vector3d(0, 1, 0);
static constexpr double kDefaultWindDirectionVariance = 0.0;
static constexpr double kDefaultWindGustDirectionVariance = 0.0;

/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboWindPlugin : public ModelPlugin {
 public:
  GazeboWindPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_("wind"),
        wind_force_mean_(kDefaultWindForceMean),
        wind_force_max_(kDefaultWindForceMax),
        wind_force_variance_(kDefaultWindForceVariance),
        wind_gust_force_mean_(kDefaultWindGustForceMean),
        wind_gust_force_max_(kDefaultWindGustForceMax),
        wind_gust_force_variance_(kDefaultWindGustForceVariance),
        wind_direction_mean_(kDefaultWindDirectionMean),
        wind_direction_variance_(kDefaultWindDirectionVariance),
        wind_gust_direction_mean_(kDefaultWindGustDirectionMean),
        wind_gust_direction_variance_(kDefaultWindGustDirectionVariance),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(NULL) {}

  virtual ~GazeboWindPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_pub_topic_;

  double wind_force_mean_;
  double wind_force_max_;
  double wind_force_variance_;
  double wind_gust_force_mean_;
  double wind_gust_force_max_;
  double wind_gust_force_variance_;
  std::default_random_engine wind_force_generator_;
  std::normal_distribution<double> wind_force_distribution_;
  std::default_random_engine wind_gust_force_generator_;
  std::normal_distribution<double> wind_gust_force_distribution_;

  ignition::math::Vector3d xyz_offset_;
  ignition::math::Vector3d wind_direction_mean_;
  ignition::math::Vector3d wind_gust_direction_mean_;
  double wind_direction_variance_;
  double wind_gust_direction_variance_;
  std::default_random_engine wind_direction_generator_;
  std::normal_distribution<double> wind_direction_distribution_X_;
  std::normal_distribution<double> wind_direction_distribution_Y_;
  std::normal_distribution<double> wind_direction_distribution_Z_;
  std::default_random_engine wind_gust_direction_generator_;
  std::normal_distribution<double> wind_gust_direction_distribution_X_;
  std::normal_distribution<double> wind_gust_direction_distribution_Y_;
  std::normal_distribution<double> wind_gust_direction_distribution_Z_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr wind_pub_;

  physics_msgs::msgs::Wind wind_msg;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
