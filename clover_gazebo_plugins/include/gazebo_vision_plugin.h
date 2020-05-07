/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Vision Plugin
 *
 * This plugin simulates VIO data
 *
 * @author Christoph Tobler <christoph@px4.io>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#ifndef _GAZEBO_VISION_PLUGIN_HH_
#define _GAZEBO_VISION_PLUGIN_HH_

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Odometry.pb.h>

namespace gazebo
{
class GAZEBO_VISIBLE VisionPlugin : public ModelPlugin
{
public:
  VisionPlugin();
  virtual ~VisionPlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);
  void getSdfParams(sdf::ElementPtr sdf);

private:
  std::string _namespace;
  physics::ModelPtr _model;
  physics::WorldPtr _world;
  event::ConnectionPtr _updateConnection;

  nav_msgs::msgs::Odometry odom_msg;

  transport::NodePtr _nh;
  transport::PublisherPtr _pub_odom;

  common::Time _last_pub_time;
  common::Time _last_time;

  ignition::math::Pose3d _pose_model_start;

  double _pub_rate;
  // vision position estimate noise parameters
  double _corellation_time;
  double _random_walk;
  double _noise_density;

  ignition::math::Vector3d _bias;

  std::default_random_engine _rand;
  std::normal_distribution<float> _randn;
  static constexpr double kDefaultPubRate 		= 30.0;	 // [Hz]
  static constexpr double kDefaultCorrelationTime	= 60.0;	 // [s]
  static constexpr double kDefaultRandomWalk		= 1.0;	 // [(m/s) / sqrt(hz)]
  static constexpr double kDefaultNoiseDensity		= 0.0005;// [(m) / sqrt(hz)]

};     // class GAZEBO_VISIBLE VisionPlugin
}      // namespace gazebo
#endif // _GAZEBO_VISION_PLUGIN_HH_
