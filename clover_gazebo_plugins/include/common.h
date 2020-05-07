#ifndef SITL_GAZEBO_COMMON_H_
#define SITL_GAZEBO_COMMON_H_
/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#include <tinyxml.h>
#include <typeinfo>
#include <Eigen/Dense>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"

namespace gazebo {

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

template <typename T>
void model_param(const std::string& world_name, const std::string& model_name, const std::string& param, T& param_value)
{
  TiXmlElement* e_param = nullptr;
  TiXmlElement* e_param_tmp = nullptr;
  std::string dbg_param;

  TiXmlDocument doc(world_name + ".xml");
  if (doc.LoadFile())
  {
    TiXmlHandle h_root(doc.RootElement());

    TiXmlElement* e_model = h_root.FirstChild("model").Element();

    for( ; e_model != nullptr; e_model=e_model->NextSiblingElement("model") )
    {
      const char* attr_name = e_model->Attribute("name");
      if (attr_name)
      {
        //specific
        if (model_name.compare(attr_name) == 0)
        {
          e_param_tmp = e_model->FirstChildElement(param);
          if (e_param_tmp)
          {
            e_param = e_param_tmp;
            dbg_param = "";
          }
          break;
        }
      }
      else
      {
        //common
        e_param = e_model->FirstChildElement(param);
        dbg_param = "common ";
      }
    }

    if (e_param)
    {
      std::istringstream iss(e_param->GetText());
      iss >> param_value;

      gzdbg << model_name << " model: " << dbg_param << "parameter " << param << " = " << param_value << " from " << doc.Value() << "\n";
    }
  }

}

/**
 * \brief Get a math::Angle as an angle from [0, 360)
 */
inline double GetDegrees360(const ignition::math::Angle& angle) {
  double degrees = angle.Degree();
  while (degrees < 0.) degrees += 360.0;
  while (degrees >= 360.0) degrees -= 360.0;
  return degrees;
}


}  // namespace gazebo

template <typename T>
class FirstOrderFilter {
/*
This class can be used to apply a first order filter on a signal.
It allows different acceleration and deceleration time constants.

Short reveiw of discrete time implementation of firest order system:
Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
continous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
*/

  public:
    FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

    T updateFilter(T inputState, double samplingTime) {
      /*
      This method will apply a first order filter on the inputState.
      */
      T outputState;
      if(inputState > previousState_){
        // Calcuate the outputState if accelerating.
        double alphaUp = exp(- samplingTime / timeConstantUp_);
        // x(k+1) = Ad*x(k) + Bd*u(k)
        outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

      }else{
        // Calculate the outputState if decelerating.
        double alphaDown = exp(- samplingTime / timeConstantDown_);
        outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
      }
      previousState_ = outputState;
      return outputState;

    }
    ~FirstOrderFilter() {}

  protected:
    double timeConstantUp_;
    double timeConstantDown_;
    T previousState_;
};

/// Returns scalar value constrained by (min_val, max_val)
template<typename Scalar>
static inline constexpr const Scalar &constrain(const Scalar &val, const Scalar &min_val, const Scalar &max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/// Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

#if GAZEBO_MAJOR_VERSION < 9
inline ignition::math::Vector3d ignitionFromGazeboMath(const gazebo::math::Vector3 &vec_gz) {
  return ignition::math::Vector3d(vec_gz.x, vec_gz.y, vec_gz.z);
}

inline ignition::math::Pose3d ignitionFromGazeboMath(const gazebo::math::Pose &pose_gz) {

  return ignition::math::Pose3d(pose_gz.pos.x, pose_gz.pos.y, pose_gz.pos.z,
                                pose_gz.rot.w, pose_gz.rot.x, pose_gz.rot.y, pose_gz.rot.z);
}
#endif

/**
 * @note Frames of reference:
 * g - gazebo (ENU), east, north, up
 * r - rotors imu frame (FLU), forward, left, up
 * b - px4 (FRD) forward, right down
 * n - px4 (NED) north, east, down
 */

/**
 * @brief Quaternion for rotation between ENU and NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 */
static const auto q_ng = ignition::math::Quaterniond(0, 0.70711, 0.70711, 0);

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 */
static const auto q_br = ignition::math::Quaterniond(0, 1, 0, 0);

// sensor X-axis unit vector in `base_link` frame
static const ignition::math::Vector3d kDownwardRotation = ignition::math::Vector3d(0, 0, -1);
static const ignition::math::Vector3d kUpwardRotation = ignition::math::Vector3d(0, 0, 1);
static const ignition::math::Vector3d kBackwardRotation = ignition::math::Vector3d(-1, 0, 0);
static const ignition::math::Vector3d kForwardRotation = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kLeftRotation = ignition::math::Vector3d(0, 1, 0);
static const ignition::math::Vector3d kRightRotation = ignition::math::Vector3d(0, -1, 0);

#endif  // SITL_GAZEBO_COMMON_H_
