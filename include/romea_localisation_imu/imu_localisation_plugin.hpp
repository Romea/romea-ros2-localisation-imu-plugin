#ifndef __ImuLocalisationPlugin_HPP__
#define __ImuLocalisationPlugin_HPP__

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

//romea
#include "visibility_control.h"
#include <romea_core_localisation_imu/LocalisationIMUPlugin.hpp>
#include <romea_localisation_utils/conversions/observation_angular_speed_conversions.hpp>
#include <romea_localisation_utils/conversions/observation_attitude_conversions.hpp>
#include <romea_common_utils/conversions/diagnostic_conversions.hpp>
#include <romea_common_utils/publishers/diagnostic_publisher.hpp>

namespace romea {


class ImuLocalisationPlugin
{
public :

  using ImuMsg = sensor_msgs::msg::Imu;
  using OdometryMsg = nav_msgs::msg::Odometry;
  using ObservationAttitudeStampedMsg = romea_localisation_msgs::msg::ObservationAttitudeStamped;
  using ObservationAngularSpeedStampedMsg = romea_localisation_msgs::msg::ObservationAngularSpeedStamped;

public :

  ROMEA_LOCALISATION_IMU_PUBLIC
  ImuLocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_LOCALISATION_IMU_PUBLIC
  virtual ~ImuLocalisationPlugin()=default;

  ROMEA_LOCALISATION_IMU_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

protected:

  void declare_parameters_();

  void process_imu_(ImuMsg::ConstSharedPtr  msg);

  void process_odom_(OdometryMsg::ConstSharedPtr msg);

  void process_angular_speed_(const rclcpp::Time & stamp,const ImuMsg & msg);

  void process_attitude_(const rclcpp::Time & stamp,const ImuMsg & msg);

  void publish_angular_speed_(const rclcpp::Time & stamp,const std::string & frame_id);

  void publish_attitude_(const rclcpp::Time & stamp,const std::string & frame_id);

  void init_attitude_publisher_();

  void init_angular_speed_publisher_();

  void init_diagnostic_publisher_();

  void init_imu_subcriber_();

  void init_odometry_subscriber_();

  void init_plugin_();

  void init_debug_();

protected:

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<LocalisationIMUPlugin> plugin_;
  ObservationAngularSpeed angular_speed_observation_;
  ObservationAttitude attitude_observation_;
  std::atomic<double> speed_;

  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odo_sub_;
  rclcpp::Publisher<ObservationAttitudeStampedMsg>::SharedPtr attitude_pub_;
  rclcpp::Publisher<ObservationAngularSpeedStampedMsg>::SharedPtr angular_speed_pub_;
  std::unique_ptr<DiagnosticPublisher<DiagnosticReport>> diagnostic_pub_;

  bool restamping_;


//  IMUDiagnostic diagnostics_;

//  SimpleFileLogger debugLogger_;
};

}

#endif
