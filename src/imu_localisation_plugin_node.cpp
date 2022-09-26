#include "romea_localisation_imu/imu_localisation_plugin.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;
  romea::ImuLocalisationPlugin plugin(options);
  exec.add_node(plugin.get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
