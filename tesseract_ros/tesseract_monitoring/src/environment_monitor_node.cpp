#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_environment/core/environment.h>
#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract_environment;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string MONITOR_NAMESPACE_PARAM = "env_monitor"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("env_monitor");

  std::string robot_description{ROBOT_DESCRIPTION_PARAM};
  std::string discrete_plugin{""};
  std::string continuous_plugin{""};
  std::string joint_state_topic{""};
  std::string monitor_namespace{""};
  std::string monitored_namespace{""};
  bool publish_environment{ false };

  // ToDo: Implement params
  // node->get_parameter_or<std::string>("monitor_namespace", monitor_namespace, rclcpp::Parameter(MONITOR_NAMESPACE_PARAM);
  //
  // if (!node->has_parameter(monitor_namespace))
  // {
  //   RCLCPP_ERROR(node->get_logger(), "Missing required parameter monitor_namespace!");
  //   return 1;
  // }
  // 
  // node->get_parameter_or<std::string>("monitored_namespace", monitored_namespace, "");
  // node->get_parameter_or<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  // node->get_parameter_or<std::string>("discrete_plugin", discrete_plugin, "");
  // node->get_parameter_or<std::string>("continuous_plugin", continuous_plugin, "");
  // node->get_parameter_or<std::string>("joint_state_topic", joint_state_topic, "");
  // node->get_parameter_or<bool>("publish_environment", publish_environment, publish_environment);

  tesseract_monitoring::EnvironmentMonitor monitor(
      robot_description, node, monitor_namespace, discrete_plugin, continuous_plugin);

  if (publish_environment)
    monitor.startPublishingEnvironment();

  if (!monitored_namespace.empty())
    monitor.startMonitoringEnvironment(monitored_namespace);

  if (joint_state_topic.empty())
    monitor.startStateMonitor();
  else
    monitor.startStateMonitor(joint_state_topic);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("env_monitor");
//   auto monitor = std::make_shared<tesseract_monitoring::EnvironmentMonitor>("env", node);
//   monitor->postInitialize();
//
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   monitor.reset();
//   return 0;
// }
