#ifndef ROMEA_LOCALISATION_IMU_IMU_LOCALISATION_PLUGIN_HPP
#define ROMEA_LOCALISATION_IMU_IMU_LOCALISATION_PLUGIN_HPP

// std
#include <string>
#include <memory>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

// romea
#include "romea_localisation_imu/visibility_control.h"
#include <romea_core_localisation_imu/LocalisationIMUPlugin.hpp>
#include <romea_localisation_utils/conversions/observation_angular_speed_conversions.hpp>
#include <romea_localisation_utils/conversions/observation_attitude_conversions.hpp>
#include <romea_common_utils/conversions/diagnostic_conversions.hpp>
#include <romea_common_utils/publishers/diagnostic_publisher.hpp>

namespace romea {


class IMULocalisationPlugin
{
public :

  using ImuMsg = sensor_msgs::msg::Imu;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using ObservationAttitudeStampedMsg =
    romea_localisation_msgs::msg::ObservationAttitudeStamped;
  using ObservationAngularSpeedStampedMsg =
    romea_localisation_msgs::msg::ObservationAngularSpeedStamped;

public :

  ROMEA_LOCALISATION_IMU_PUBLIC
  explicit IMULocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_LOCALISATION_IMU_PUBLIC
  virtual ~IMULocalisationPlugin() = default;

  ROMEA_LOCALISATION_IMU_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:

  void declare_parameters_();

  void init_attitude_publisher_();

  void init_angular_speed_publisher_();

  void init_diagnostic_publisher_();

  void init_imu_subcriber_();

  void init_odometry_subscriber_();

  void init_plugin_();

  void init_debug_();

  void init_timer_();


  void process_imu_(ImuMsg::ConstSharedPtr  msg);

  void process_odom_(OdometryMsg::ConstSharedPtr msg);

  void process_angular_speed_(const rclcpp::Time & stamp, const ImuMsg & msg);

  void process_attitude_(const rclcpp::Time & stamp, const ImuMsg & msg);

  void publish_angular_speed_(const rclcpp::Time & stamp, const std::string & frame_id);

  void publish_attitude_(const rclcpp::Time & stamp, const std::string & frame_id);

  void timer_callback_();


protected:

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<LocalisationIMUPlugin> plugin_;
  ObservationAngularSpeed angular_speed_observation_;
  ObservationAttitude attitude_observation_;

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;
  rclcpp::Publisher<ObservationAttitudeStampedMsg>::SharedPtr attitude_pub_;
  rclcpp::Publisher<ObservationAngularSpeedStampedMsg>::SharedPtr angular_speed_pub_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool restamping_;
  bool enable_accelerations_;
};

}  // namespace romea

#endif  // ROMEA_LOCALISATION_IMU_IMU_LOCALISATION_PLUGIN_HPP
