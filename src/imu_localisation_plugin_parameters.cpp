// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// romea
#include "romea_localisation_imu_plugin/imu_localisation_plugin_parameters.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"

namespace
{

const char restamping_param_name[] = "restamping";
const char enable_accelerations_param_name[] = "enable_accelerations";

const char rate_param_name[] = "imu.rate";
const char acceleration_noise_density_param_name[] = "imu.acceleration_noise_density";
const char acceleration_bias_stability_std_param_name[] = "imu.acceleration_bias_stability_std";
const char acceleration_range_param_name[] = "imu.acceleration_range";
const char angular_speed_noise_density_param_name[] = "imu.angular_speed_noise_density";
const char angular_speed_bias_stability_std_param_name[] = "imu.angular_speed_bias_stability_std";
const char angular_speed_range_param_name[] = "imu.angular_speed_range";
const char magnetic_noise_density_param_name[] = "imu.magnetic_noise_density";
const char magnetic_bias_stability_std_param_name[] = "imu.magnetic_bias_stability_std";
const char magnetic_range_param_name[] = "imu.magnetic_range";
const char heading_std_param_name[] = "imu.heading_std";
}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
void declare_restamping(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<bool>(node, restamping_param_name, false);
}

//-----------------------------------------------------------------------------
void declare_enable_accelerations(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<bool>(node, enable_accelerations_param_name, true);
}

//----------------------------------------------------------------------------
bool get_restamping(rclcpp::Node::SharedPtr node)
{
  return get_parameter<bool>(node, restamping_param_name);
}

//----------------------------------------------------------------------------
bool get_enable_accelerations(rclcpp::Node::SharedPtr node)
{
  return get_parameter<bool>(node, enable_accelerations_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_rate(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, rate_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_acceleration_noise_density(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, acceleration_noise_density_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_acceleration_bias_stability_std(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, acceleration_bias_stability_std_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_acceleration_range(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, acceleration_range_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_angular_speed_noise_density(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, angular_speed_noise_density_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_angular_speed_bias_stability_std(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, angular_speed_bias_stability_std_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_angular_speed_range(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, angular_speed_range_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_magnetic_noise_density(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, magnetic_noise_density_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_magnetic_bias_stability_std(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, magnetic_bias_stability_std_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_magnetic_range(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, magnetic_range_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_heading_std(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, heading_std_param_name);
}

//-----------------------------------------------------------------------------
void declare_imu_body_pose(rclcpp::Node::SharedPtr node)
{
  declare_eigen_rigid_transformation_parameter<Eigen::Affine3d>(node, "imu");
}

//-----------------------------------------------------------------------------
double get_imu_rate(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, rate_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_acceleration_noise_density(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, acceleration_noise_density_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_acceleration_bias_stability_std(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, acceleration_bias_stability_std_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_acceleration_range(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, acceleration_range_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_angular_speed_noise_density(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, angular_speed_noise_density_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_angular_speed_bias_stability_std(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, angular_speed_bias_stability_std_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_angular_speed_range(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, angular_speed_range_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_magnetic_noise_density(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, magnetic_noise_density_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_magnetic_bias_stability_std(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, magnetic_bias_stability_std_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_magnetic_range(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, magnetic_range_param_name);
}

//-----------------------------------------------------------------------------
double get_imu_heading_std(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, heading_std_param_name);
}

//-----------------------------------------------------------------------------
Eigen::Affine3d get_imu_body_pose(rclcpp::Node::SharedPtr node)
{
  return get_eigen_rigid_transformation_parameter<Eigen::Affine3d>(node, "imu");
}

}  // namespace romea
