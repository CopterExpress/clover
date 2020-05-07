/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
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
/**
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#ifndef _GAZEBO_GPS_PLUGIN_HH_
#define _GAZEBO_GPS_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <SITLGps.pb.h>
#include <Groundtruth.pb.h>

namespace gazebo
{
class GAZEBO_VISIBLE GpsPlugin : public ModelPlugin
{
public:
  GpsPlugin();
  virtual ~GpsPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

protected:
  /* Keep this protected so that it's possible to unit test it. */
  std::pair<double, double> reproject(ignition::math::Vector3d& pos);

private:
  std::string namespace_;
  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  bool gps_noise_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr gt_pub_;
  transport::PublisherPtr gps_pub_;

  sensor_msgs::msgs::SITLGps gps_msg;
  sensor_msgs::msgs::Groundtruth groundtruth_msg;

  common::Time last_gps_time_;
  common::Time last_time_;

  // Set global reference point
  // Zurich Irchel Park: 47.397742, 8.545594, 488m
  // Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
  // Moscow downtown: 55.753395, 37.625427, 155m

  // The home position can be specified using the environment variables:
  // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT

  // Zurich Irchel Park
  double lat_home = 47.397742 * M_PI / 180.0;  // rad
  double lon_home = 8.545594 * M_PI / 180.0;   // rad
  double alt_home = 488.0;                     // meters
  // Seattle downtown (15 deg declination): 47.592182, -122.316031
  // static const double lat_home = 47.592182 * M_PI / 180;    // rad
  // static const double lon_home = -122.316031 * M_PI / 180;  // rad
  // static const double alt_home = 86.0;                      // meters

  static constexpr const double earth_radius = 6353000.0;      // meters

  // gps delay related
  static constexpr double gps_update_interval_ = 0.2; // 5hz
  static constexpr double gps_delay = 0.12;           // 120 ms
  static constexpr int gps_buffer_size_max = 1000;
  std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer;

  ignition::math::Vector3d gps_bias;
  ignition::math::Vector3d noise_gps_pos;
  ignition::math::Vector3d noise_gps_vel;
  ignition::math::Vector3d random_walk_gps;
  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  // gps noise parameters
  double std_xy;    // meters
  double std_z;     // meters
  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;
  static constexpr double gps_corellation_time = 60.0;    // s
  static constexpr double gps_xy_random_walk = 2.0;       // (m/s) / sqrt(hz)
  static constexpr double gps_z_random_walk = 4.0;        // (m/s) / sqrt(hz)
  static constexpr double gps_xy_noise_density = 2e-4;    // (m) / sqrt(hz)
  static constexpr double gps_z_noise_density = 4e-4;     // (m) / sqrt(hz)
  static constexpr double gps_vxy_noise_density = 2e-1;   // (m/s) / sqrt(hz)
  static constexpr double gps_vz_noise_density = 4e-1;    // (m/s) / sqrt(hz)
};     // class GAZEBO_VISIBLE GpsPlugin
}      // namespace gazebo
#endif // _GAZEBO_GPS_PLUGIN_HH_
