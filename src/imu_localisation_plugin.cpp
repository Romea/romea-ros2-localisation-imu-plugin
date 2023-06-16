// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <string>
#include <memory>
#include <utility>

// romea
#include "romea_localisation_imu/imu_localisation_plugin_parameters.hpp"
#include "romea_localisation_imu/imu_localisation_plugin.hpp"

#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"
#include "romea_common_utils/params/algorithm_parameters.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_common_utils/conversions/geometry_conversions.hpp"
#include "romea_common_utils/qos.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
IMULocalisationPlugin::IMULocalisationPlugin(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("imu_localisation_plugin", options)),
  plugin_(nullptr),
  angular_speed_observation_(),
  attitude_observation_(),
  imu_sub_(nullptr),
  odom_sub_(nullptr),
  attitude_pub_(nullptr),
  angular_speed_pub_(nullptr),
  diagnostic_pub_(nullptr),
  timer_(nullptr),
  restamping_(false)
{
  declare_parameters_();
  init_plugin_();
  init_debug_();
  init_attitude_publisher_();
  init_angular_speed_publisher_();
  init_diagnostic_publisher_();
  init_imu_subcriber_();
  init_odometry_subscriber_();
  init_timer_();
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
IMULocalisationPlugin::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::declare_parameters_()
{
  declare_imu_rate(node_);
  declare_imu_acceleration_noise_density(node_);
  declare_imu_acceleration_bias_stability_std(node_);
  declare_imu_acceleration_range(node_);
  declare_imu_angular_speed_noise_density(node_);
  declare_imu_angular_speed_bias_stability_std(node_);
  declare_imu_angular_speed_range(node_);
  declare_imu_magnetic_noise_density(node_);
  declare_imu_magnetic_bias_stability_std(node_);
  declare_imu_magnetic_range(node_);
  declare_imu_heading_std(node_);
  declare_imu_body_pose(node_);

  declare_enable_accelerations(node_);
  declare_restamping(node_);
  declare_debug(node_);
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_angular_speed_publisher_()
{
  angular_speed_pub_ = node_->create_publisher<ObservationAngularSpeedStampedMsg>(
    "angular_speed", sensor_data_qos());

}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_attitude_publisher_()
{
  attitude_pub_ = node_->create_publisher<ObservationAttitudeStampedMsg>(
    "attitude", sensor_data_qos());
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_diagnostic_publisher_()
{
  diagnostic_pub_ = make_diagnostic_publisher<DiagnosticReport>(node_, node_->get_name(), 1.0);
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_imu_subcriber_()
{
  auto callback = std::bind(&IMULocalisationPlugin::process_imu_, this, std::placeholders::_1);

  imu_sub_ = node_->create_subscription<ImuMsg>(
    "imu/data", best_effort(1), callback);
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_odometry_subscriber_()
{
  auto callback = std::bind(&IMULocalisationPlugin::process_odom_, this, std::placeholders::_1);

  odom_sub_ = node_->create_subscription<OdometryMsg>(
    "vehicle_controller/odom", best_effort(1), callback);
}


//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_plugin_()
{
  auto imu = std::make_unique<IMUAHRS>(
    get_imu_rate(node_),
    get_imu_acceleration_noise_density(node_),
    get_imu_acceleration_bias_stability_std(node_),
    get_imu_acceleration_range(node_),
    get_imu_angular_speed_noise_density(node_),
    get_imu_angular_speed_bias_stability_std(node_),
    get_imu_angular_speed_range(node_),
    get_imu_magnetic_noise_density(node_),
    get_imu_magnetic_bias_stability_std(node_),
    get_imu_magnetic_range(node_),
    get_imu_heading_std(node_));

  imu->setBodyPose(get_imu_body_pose(node_));

  plugin_ = std::make_unique<LocalisationIMUPlugin>(std::move(imu));
  enable_accelerations_ = get_enable_accelerations(node_);
  restamping_ = get_restamping(node_);
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_debug_()
{
  if (get_debug(node_)) {
    plugin_->enableDebugLog(get_log_filename(node_));
  }
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::init_timer_()
{
  auto callback = std::bind(&IMULocalisationPlugin::timer_callback_, this);
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), callback);
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::process_odom_(OdometryMsg::ConstSharedPtr msg)
{
  // std::cout << " processOdom" << std::endl;
  auto stamp = restamping_ ? node_->get_clock()->now() :
    rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);

  plugin_->processLinearSpeed(
    to_romea_duration(stamp),
    std::sqrt(
      msg->twist.twist.linear.x *
      msg->twist.twist.linear.x +
      msg->twist.twist.linear.y *
      msg->twist.twist.linear.y));
}


//-----------------------------------------------------------------------------
void IMULocalisationPlugin::process_imu_(ImuMsg::ConstSharedPtr msg)
{
  // std::cout << " processIMU " << std::endl;

  auto stamp = restamping_ ? node_->get_clock()->now() :
    rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);

  process_attitude_(stamp, *msg);
  process_angular_speed_(stamp, *msg);
}


//-----------------------------------------------------------------------------
void IMULocalisationPlugin::process_angular_speed_(
  const rclcpp::Time & stamp,
  const ImuMsg & msg)
{
  if (plugin_->computeAngularSpeed(
      to_romea_duration(stamp),
      enable_accelerations_ * msg.linear_acceleration.x,
      enable_accelerations_ * msg.linear_acceleration.y,
      enable_accelerations_ * msg.linear_acceleration.z,
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z,
      angular_speed_observation_))
  {
    publish_angular_speed_(stamp, msg.header.frame_id);
  }
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::process_attitude_(
  const rclcpp::Time & stamp,
  const ImuMsg & msg)
{
  Eigen::Quaterniond q;
  to_romea(msg.orientation, q);
  Eigen::Vector3d orientation = quaternionToEulerAngles(q);

  if (plugin_->computeAttitude(
      to_romea_duration(stamp),
      orientation.x(),
      orientation.y(),
      orientation.z(),
      attitude_observation_))
  {
    publish_attitude_(stamp, msg.header.frame_id);
  }
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::publish_angular_speed_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto angular_speed_msg = std::make_unique<ObservationAngularSpeedStampedMsg>();
  to_ros_msg(stamp, frame_id, angular_speed_observation_, *angular_speed_msg);
  angular_speed_pub_->publish(std::move(angular_speed_msg));
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::publish_attitude_(
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  auto attitude_msg = std::make_unique<ObservationAttitudeStampedMsg>();
  to_ros_msg(stamp, frame_id, attitude_observation_, *attitude_msg);
  attitude_pub_->publish(std::move(attitude_msg));
}

//-----------------------------------------------------------------------------
void IMULocalisationPlugin::timer_callback_()
{
  auto stamp = node_->get_clock()->now();
  diagnostic_pub_->publish(stamp, plugin_->makeDiagnosticReport(to_romea_duration(stamp)));
}

}  // namespace romea

//-----------------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::IMULocalisationPlugin)
