//romea
#include "romea_localisation_imu/imu_parameters.hpp"
#include "romea_localisation_imu/imu_localisation_plugin.hpp"

#include <romea_core_common/math/EulerAngles.hpp>
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_common_utils/conversions/time_conversions.hpp>
#include <romea_common_utils/conversions/geometry_conversions.hpp>
#include <romea_common_utils/qos.hpp>


namespace romea {

//-----------------------------------------------------------------------------
ImuLocalisationPlugin::ImuLocalisationPlugin(const rclcpp::NodeOptions & options):
  node_(std::make_shared<rclcpp::Node>("imu_localisation_plugin", options)),
  plugin_(nullptr),
  angular_speed_observation_(),
  attitude_observation_(),
  speed_(),
  imu_sub_(nullptr),
  odo_sub_(nullptr),
  attitude_pub_(nullptr),
  angular_speed_pub_(nullptr),
  diagnostic_pub_(nullptr),
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
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
ImuLocalisationPlugin::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::declare_parameters_()
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
  declare_parameter_with_default<bool>(node_,"restamping",false);
  declare_parameter_with_default<bool>(node_,"debug",false);
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::init_angular_speed_publisher_()
{
  node_->create_publisher<ObservationAngularSpeedStampedMsg>("angular_speed",sensor_data_qos());
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::init_attitude_publisher_()
{
  node_->create_publisher<ObservationAttitudeStampedMsg>("attitude",sensor_data_qos());
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::init_diagnostic_publisher_()
{
  diagnostic_pub_ = std::make_unique<DiagnosticPublisher<DiagnosticReport>>(node_,node_->get_name(),1.0);
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::init_imu_subcriber_()
{
  auto callback = std::bind(&ImuLocalisationPlugin::process_imu_, this, std::placeholders::_1);
  node_->create_subscription<ImuMsg>("imu/data",best_effort(1),callback);
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::init_odometry_subscriber_()
{
  auto callback = std::bind(&ImuLocalisationPlugin::process_odom_, this, std::placeholders::_1);
  node_->create_subscription<OdometryMsg>("vehicle_controller/odom",best_effort(1),callback);
}


//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::init_plugin_()
{
  auto imu = std::make_unique<IMUAHRS>(get_imu_rate(node_),
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

  plugin_ = std::make_unique<LocalisationIMUPlugin>(std::move(imu));
  restamping_ = get_parameter<bool>(node_,"restamping");
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::init_debug_()
{
  if(get_parameter<bool>(node_,"debug"))
  {
    std::string filename = std::string(node_->get_fully_qualified_name())+"/debug.dat";
    std::replace_copy(std::begin(filename)+1, std::end(filename), std::begin(filename)+1, '/', '-');
    plugin_->enableDebugLog(rclcpp::get_logging_directory().string()+filename);
  }
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::process_odom_(OdometryMsg::ConstSharedPtr msg)
{
  //std::cout << "processOdom" << std::endl;
  speed_.store(std::sqrt(msg->twist.twist.linear.x*
                         msg->twist.twist.linear.x+
                         msg->twist.twist.linear.y*
                         msg->twist.twist.linear.y));
}


//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::process_imu_(ImuMsg::ConstSharedPtr msg)
{
  //  std::cout << " processIMU" << std::endl;

  auto stamp = restamping_ ? node_->get_clock()->now() : rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);

  process_attitude_(stamp,*msg);
  process_angular_speed_(stamp,*msg);
  diagnostic_pub_->publish(stamp,plugin_->makeDiagnosticReport());
}


//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::process_angular_speed_(const rclcpp::Time &stamp,
                                                   const ImuMsg & msg)
{
  if(plugin_->computeAngularSpeed(to_romea_duration(stamp),
                                  speed_,
                                  msg.linear_acceleration.x,
                                  msg.linear_acceleration.y,
                                  msg.linear_acceleration.z,
                                  msg.angular_velocity.x,
                                  msg.angular_velocity.y,
                                  msg.angular_velocity.z,
                                  angular_speed_observation_))
  {
    publish_angular_speed_(stamp,msg.header.frame_id);
  }

}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::process_attitude_(const rclcpp::Time & stamp,
                                              const ImuMsg & msg)
{

  Eigen::Quaterniond q;
  to_romea(msg.orientation,q);
  Eigen::Vector3d orientation = quaternionToEulerAngles(q);

  if(plugin_->computeAttitude(to_romea_duration(stamp),
                              orientation.x(),
                              orientation.y(),
                              orientation.z(),
                              attitude_observation_))
  {
    publish_attitude_(stamp,msg.header.frame_id);
  }

}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::publish_angular_speed_(const rclcpp::Time & stamp,
                                                   const std::string & frame_id)
{
  using namespace romea_localisation_msgs::msg;
  auto angular_speed_msg = std::make_unique<ObservationAngularSpeedStamped>();
  to_ros_msg(stamp,frame_id,angular_speed_observation_,*angular_speed_msg);
  angular_speed_pub_->publish(std::move(angular_speed_msg));
}

//-----------------------------------------------------------------------------
void ImuLocalisationPlugin::publish_attitude_(const rclcpp::Time & stamp,
                                              const std::string & frame_id)
{
  using namespace romea_localisation_msgs::msg;
  auto attitude_msg = std::make_unique<ObservationAttitudeStamped>();
  to_ros_msg(stamp,frame_id,attitude_observation_,*attitude_msg);
  attitude_pub_->publish(std::move(attitude_msg));
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ImuLocalisationPlugin)

