// #include <tesseract_common/macros.h>
#include <tesseract_monitoring/contact_monitor.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <pluginlib/class_loader.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <tesseract_msgs/msg/contact_result_vector.hpp>
// #include <tesseract_msgs/srv/modify_environment.hpp>
// #include <tesseract_msgs/srv/compute_contact_result_vector.hpp>
// #include <boost/thread.hpp>
// #include <boost/thread/shared_mutex.hpp>
// #include <boost/thread/recursive_mutex.hpp>
#include <tesseract_msgs/msg/contact_result_vector.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

// #include <tesseract_collision/core/discrete_contact_manager.h>
// #include <tesseract/tesseract.h>
// #include <tesseract_scene_graph/graph.h>
// #include <tesseract_scene_graph/utils.h>
// #include <tesseract_scene_graph/resource_locator.h>
// #include <tesseract_urdf/urdf_parser.h>
// #include <tesseract_scene_graph/srdf_model.h>
// #include <tesseract_environment/kdl/kdl_env.h>
// #include <tesseract_environment/core/utils.h>
 #include <tesseract_rosutils/utils.h>
 #include <tesseract_rosutils/plotting.h>
 #include <tesseract_monitoring/constants.h>
// #include <tesseract_environment/core/environment.h>
// #include <tesseract_monitoring/environment_monitor.h>

namespace tesseract_monitoring
{
ContactMonitor::ContactMonitor(std::string monitor_namespace,
                               const tesseract_environment::Environment::Ptr& env,
                               rclcpp::Node& node,
                               rclcpp::Node& pnh,
                               const std::vector<std::string>& monitored_link_names,
                               const tesseract_collision::ContactTestType& type,
                               double contact_distance,
                               const std::string& joint_state_topic)
  : monitor_namespace_(std::move(monitor_namespace))
  , env_(env)
  , node_(node)
  , pnh_(pnh)
  , monitored_link_names_(monitored_link_names)
  , type_(type)
  , contact_distance_(contact_distance)
{
  if (env_ == nullptr)
  {
    RCLCPP_ERROR(node->get_logger(),"Null pointer passed for environment object.  Not setting up contact monitor.");
    return;
  }

  // Create Environment Monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, monitor_namespace_);

  manager_ = env_->getDiscreteContactManager();
  if (manager_ == nullptr)
  {
    RCLCPP_ERROR(node_->get_logger(),"Discrete contact manager is a null pointer.  Not setting up contact monitor.");
    return;
  }
  manager_->setActiveCollisionObjects(monitored_link_names);
  manager_->setDefaultCollisionMarginData(contact_distance);

  joint_states_sub_ =
        node_->create_subscription<sensor_msgs::msg::JointState>(joint_state_topic, 1,  &ContactMonitor::callbackJointState);
  std::string contact_results_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_CONTACT_RESULTS_TOPIC;
  std::string compute_contact_results = R"(/)" + monitor_namespace_ + DEFAULT_COMPUTE_CONTACT_RESULTS_SERVICE;

  contact_results_pub_ = node_->create_publisher<tesseract_msgs::ContactResultVector>(contact_results_topic, 1);
  compute_contact_results = node_->create_service<tesseract_msgs::srv::ComputeContactResultVector>(
         "compute_contact_results", &ContactMonitor::callbackComputeContactResultVector);
}

ContactMonitor::~ContactMonitor() { current_joint_states_evt_.notify_all(); }

void ContactMonitor::startPublishingEnvironment() { monitor_->startPublishingEnvironment(); }

void ContactMonitor::startMonitoringEnvironment(const std::string& monitored_namepsace)
{
  monitor_->startMonitoringEnvironment(monitored_namepsace);
}

void ContactMonitor::startPublishingMarkers()
{
  publish_contact_markers_ = true;
  std::string contact_marker_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_CONTACT_MARKER_TOPIC;
  contact_marker_pub_ = pnh_->create_publisher<visualization_msgs::msg::MarkerArray>(contact_marker_topic, 1);
}

/**
 * @brief Compute collision results and publish results.
 *
 * This also publishes environment and contact markers if correct flags are enabled for visualization and debuging.
 */
void ContactMonitor::computeCollisionReportThread()
{
  while (!ros::isShuttingDown())
  {
    boost::shared_ptr<sensor_msgs::msg::JointState> msg = nullptr;
    tesseract_collision::ContactResultMap contacts;
    tesseract_msgs::msg::ContactResultVector contacts_msg;
    // Limit the lock
    {
      std::unique_lock lock(modify_mutex_);
      if (env_revision_ != env_->getRevision())
      {
        env_revision_ = env_->getRevision();

        // Create a new manager
        std::vector<std::string> active = manager_->getActiveCollisionObjects();
        tesseract_collision::CollisionMarginData contact_margin_data = manager_->getCollisionMarginData();
        tesseract_collision::IsContactAllowedFn fn = manager_->getIsContactAllowedFn();

        manager_ = env_->getDiscreteContactManager();
        manager_->setActiveCollisionObjects(active);
        manager_->setCollisionMarginData(contact_margin_data);
        manager_->setIsContactAllowedFn(fn);
      }

      if (!current_joint_states_)
      {
        current_joint_states_evt_.wait(lock);
      }

      if (!current_joint_states_)
        continue;

      msg = current_joint_states_;
      current_joint_states_.reset();

      contacts.clear();
      contacts_msg.contacts.clear();

      env_->setState(msg->name, msg->position);
      tesseract_environment::EnvState::ConstPtr state = env_->getCurrentState();

      manager_->setCollisionObjectsTransform(state->link_transforms);
      manager_->contactTest(contacts, type_);
    }

    tesseract_collision::ContactResultVector contacts_vector;
    tesseract_collision::flattenResults(std::move(contacts), contacts_vector);
    contacts_msg.contacts.reserve(contacts_vector.size());
    for (std::size_t i = 0; i < contacts_vector.size(); ++i)
    {
      tesseract_msgs::msg::ContactResult contact_msg;
      tesseract_rosutils::toMsg(contact_msg, contacts_vector[i], msg->header.stamp);
      contacts_msg.contacts.push_back(contact_msg);
    }
    contact_results_pub_->publish(contacts_msg);

    if (publish_contact_markers_)
    {
      int id_counter = 0;
      tesseract_visualization::ContactResultsMarker marker(
          monitored_link_names_, contacts_vector, manager_->getCollisionMarginData());
      visualization_msgs::msg::MarkerArray marker_msg = tesseract_rosutils::ROSPlotting::getContactResultsMarkerArrayMsg(
          id_counter, env_->getSceneGraph()->getRoot(), "contact_monitor", msg->header.stamp, marker);
      contact_marker_pub_->publish(marker_msg);
    }
  }
}

void ContactMonitor::callbackJointState(boost::shared_ptr<sensor_msgs::JointState> msg)
{
  std::scoped_lock lock(modify_mutex_);
  current_joint_states_ = std::move(msg);
  current_joint_states_evt_.notify_all();
}

bool ContactMonitor::callbackModifyTesseractEnv(tesseract_msgs::srv::ModifyEnvironment::Request& request,
                                                tesseract_msgs::srv::ModifyEnvironment::Response& response)
{
  std::scoped_lock lock(modify_mutex_);

  int revision = static_cast<int>(request.revision);
  if (request.append)
    revision = env_->getRevision();

  if (!env_ || request.id != env_->getName() || revision != env_->getRevision())
    return false;

  response.success = tesseract_rosutils::processMsg(*env_, request.commands);
  response.revision = static_cast<unsigned long>(env_->getRevision());

  // Create a new manager
  std::vector<std::string> active = manager_->getActiveCollisionObjects();
  tesseract_collision::CollisionMarginData contact_margin_data = manager_->getCollisionMarginData();
  tesseract_collision::IsContactAllowedFn fn = manager_->getIsContactAllowedFn();

  manager_ = env_->getDiscreteContactManager();
  manager_->setActiveCollisionObjects(active);
  manager_->setCollisionMarginData(contact_margin_data);
  manager_->setIsContactAllowedFn(fn);

  return true;
}

bool ContactMonitor::callbackComputeContactResultVector(tesseract_msgs::srv::ComputeContactResultVector::Request& request,
                                                        tesseract_msgs::srv::ComputeContactResultVector::Response& response)
{
  tesseract_collision::ContactResultMap contact_results;

  // Limit the lock
  {
    std::scoped_lock lock(modify_mutex_);

    env_->setState(request.joint_states.name, request.joint_states.position);
    tesseract_environment::EnvState::ConstPtr state = env_->getCurrentState();

    manager_->setCollisionObjectsTransform(state->link_transforms);
    manager_->contactTest(contact_results, type_);
  }

  tesseract_collision::ContactResultVector contacts_vector;
  tesseract_collision::flattenResults(std::move(contact_results), contacts_vector);
  response.collision_result.contacts.reserve(contacts_vector.size());
  for (const auto& contact : contacts_vector)
  {
    tesseract_msgs::msg::ContactResult contact_msg;
    tesseract_rosutils::toMsg(contact_msg, contact, request.joint_states.header.stamp);
    response.collision_result.contacts.push_back(contact_msg);
  }
  response.success = true;

  return true;
}

}  // namespace tesseract_monitoring


// // TODO Joe: Reimplement all this as a class with member functions
//
// // using namespace tesseract;
// using namespace tesseract_environment;
// using namespace tesseract_rosutils;
// using namespace tesseract_collision;
// using namespace tesseract_scene_graph;
//
// // typedef pluginlib::ClassLoader<DiscreteContactManager> DiscreteContactManagerPluginLoader;
// // typedef std::shared_ptr<DiscreteContactManagerPluginLoader> DiscreteContactManagerPluginLoaderPtr;
//
// const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
//
// const double DEFAULT_CONTACT_DISTANCE = 0.1;
//
// //static Tesseract::Ptr tess;
// static DiscreteContactManager::Ptr manager;
//
// static rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
// static rclcpp::Publisher<tesseract_msgs::msg::ContactResultVector>::SharedPtr contact_results_pub;
// static rclcpp::Publisher<tesseract_msgs::msg::TesseractState>::SharedPtr environment_pub;
// static rclcpp::Subscription<tesseract_msgs::msg::TesseractState>::SharedPtr environment_diff_sub;
//
// static rclcpp::Service<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_env_service;
// static rclcpp::Service<tesseract_msgs::srv::ComputeContactResultVector>::SharedPtr compute_contact_results;
//
// static ContactTestType type;
// static ContactResultMap contacts;
// static tesseract_msgs::msg::ContactResultVector contacts_msg;
// static bool publish_environment;
// static boost::mutex modify_mutex;
// //static DiscreteContactManagerPluginLoaderPtr discrete_manager_loader; /**< The discrete contact manager loader */
//
// tesseract_environment::Environment::Ptr env_;
//
// void callbackJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
// {
//   boost::mutex::scoped_lock(modify_mutex);
//   contacts.clear();
//   contacts_msg.contacts.clear();
//
//   tess->getEnvironment()->setState(msg->name, msg->position);
//   EnvState::ConstPtr state = tess->getEnvironment()->getCurrentState();
//
//   manager->setCollisionObjectsTransform(state->link_transforms);
//   manager->contactTest(contacts, type);
//
//   if (publish_environment)
//   {
//     tesseract_msgs::msg::TesseractState state_msg;
//     toMsg(state_msg, *(tess->getEnvironment()));
//     environment_pub->publish(state_msg);
//   }
//
//   ContactResultVector contacts_vector;
//   tesseract_collision::flattenResults(std::move(contacts), contacts_vector);
//   contacts_msg.contacts.reserve(contacts_vector.size());
//   for (const auto& contact : contacts_vector)
//   {
//     tesseract_msgs::msg::ContactResult contact_msg;
//     toMsg(contact_msg, contact);
//     contacts_msg.contacts.push_back(contact_msg);
//   }
//   contact_results_pub->publish(contacts_msg);
// }
//
// void callbackModifyTesseractEnv(const tesseract_msgs::srv::ModifyEnvironment::Request::SharedPtr request,
//                                 tesseract_msgs::srv::ModifyEnvironment::Response::SharedPtr response)
// {
//   boost::mutex::scoped_lock(modify_mutex);
//   response->success = processMsg(*(tess->getEnvironment()), request->commands);
//   response->revision = tess->getEnvironmentConst()->getRevision();
//
//   // Create a new manager
//   std::vector<std::string> active = manager->getActiveCollisionObjects();
//   double contact_distance = manager->getContactDistanceThreshold();
//   IsContactAllowedFn fn = manager->getIsContactAllowedFn();
//
//   manager = tess->getEnvironment()->getDiscreteContactManager();
//   manager->setActiveCollisionObjects(active);
//   manager->setContactDistanceThreshold(contact_distance);
//   manager->setIsContactAllowedFn(fn);
//
//   return;
// }
//
// void callbackComputeContactResultVector(
//     const tesseract_msgs::srv::ComputeContactResultVector::Request::SharedPtr request,
//     tesseract_msgs::srv::ComputeContactResultVector::Response::SharedPtr response)
// {
//   ContactResultMap contacts;
//
//   boost::mutex::scoped_lock(modify_mutex);
//
//   tess->getEnvironment()->setState(request->joint_states.name, request->joint_states.position);
//   EnvState::ConstPtr state = tess->getEnvironment()->getCurrentState();
//
//   manager->setCollisionObjectsTransform(state->link_transforms);
//   manager->contactTest(contacts, type);
//
//   ContactResultVector contacts_vector;
//   tesseract_collision::flattenResults(std::move(contacts), contacts_vector);
//   response->collision_result.contacts.reserve(contacts_vector.size());
//   for (const auto& contact : contacts_vector)
//   {
//     tesseract_msgs::msg::ContactResult contact_msg;
//     toMsg(contact_msg, contact);
//     response->collision_result.contacts.push_back(contact_msg);
//   }
//   response->success = true;
//
//   return;
// }
//
// void callbackTesseractEnvDiff(const tesseract_msgs::msg::TesseractState::SharedPtr state)
// {
//   //  if (!state->is_diff)
//   //    return;
//
//   boost::mutex::scoped_lock(modify_mutex);
//   if (!processMsg(*(tess->getEnvironment()), *state))
//   {
//     //    ROS_ERROR("Invalid TesseractState diff message");  // TODO Joe: re-enable
//   }
//
//   // Create a new manager
//   std::vector<std::string> active = manager->getActiveCollisionObjects();
//   double contact_distance = manager->getContactDistanceThreshold();
//   IsContactAllowedFn fn = manager->getIsContactAllowedFn();
//
//   manager = tess->getEnvironment()->getDiscreteContactManager();
//   manager->setActiveCollisionObjects(active);
//   manager->setContactDistanceThreshold(contact_distance);
//   manager->setIsContactAllowedFn(fn);
// }
//
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("tesseract_contact_monitoring");
//
//   SceneGraph::Ptr scene_graph;
//   SRDFModel::Ptr srdf_model;
//   std::string robot_description;
//   // std::string plugin;
//
//   node->declare_parameter("robot_description");
//   node->declare_parameter("publish_environment");
//
//   node->get_parameter_or("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
//   node->get_parameter_or("publish_environment", publish_environment, false);
//
//   node->get_parameter_or("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
//   node->get_parameter_or("publish_environment", publish_environment, false);
//
//   //  if (pnh.hasParam("plugin"))
//   //  {
//   //    discrete_manager_loader.reset(new DiscreteContactManagerPluginLoader("tesseract_collision",
//   //    "tesseract_collision::DiscreteContactManager")); pnh.getParam("plugin", plugin); if
//   //    (discrete_manager_loader->isClassAvailable(plugin))
//   //    {
//   //      ROS_ERROR("Failed to load tesseract contact checker plugin: %s.", plugin.c_str());
//   //      return -1;
//   //    }
//   //    auto fn = [&]() -> DiscreteContactManagerPtr { return ::discrete_manager_loader->createUniqueInstance(plugin);
//   //    }; env->registerDiscreteContactManager(plugin, fn); env->setActiveDiscreteContactManager(plugin);
//   //  }
//
//   // Initial setup
//   std::string urdf_xml_string, srdf_xml_string;
//   if (!node->has_parameter(robot_description))
//   {
//     RCLCPP_ERROR(node->get_logger(), "Failed to find parameter: %s", robot_description.c_str());
//     return -1;
//   }
//
//   if (!node->has_parameter(robot_description + "_semantic"))
//   {
//     RCLCPP_ERROR(node->get_logger(), "Failed to find parameter: %s", (robot_description + "_semantic").c_str());
//     return -1;
//   }
//
//   node->get_parameter(robot_description, urdf_xml_string);
//   node->get_parameter(robot_description + "_semantic", srdf_xml_string);
//
//   // tess = std::make_shared<tesseract::Tesseract>();
//   tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
//   if (!tess->init(urdf_xml_string, srdf_xml_string, locator))
//   {
//     RCLCPP_ERROR(node->get_logger(), "Failed to initialize environment.");
//     return -1;
//   }
//
//   // Setup request information
//   std::vector<std::string> link_names;
//   double contact_distance;
//
//   node->get_parameter_or("contact_distance", contact_distance, DEFAULT_CONTACT_DISTANCE);
//
//   link_names = tess->getEnvironment()->getLinkNames();
//   if (node->has_parameter("monitor_links"))
//     node->get_parameter("monitor_links", link_names);
//
//   if (link_names.empty())
//     link_names = tess->getEnvironment()->getLinkNames();
//
//   int contact_test_type = 2;
//   if (node->has_parameter("contact_test_type"))
//     node->get_parameter("contact_test_type", contact_test_type);
//
//   if (contact_test_type < 0 || contact_test_type > 3)
//   {
//     RCLCPP_WARN(node->get_logger(), "Request type must be 0, 1, 2 or 3. Setting to 2(ALL)!");
//     contact_test_type = 2;
//   }
//   type = static_cast<ContactTestType>(contact_test_type);
//
//   manager = tess->getEnvironment()->getDiscreteContactManager();
//   manager->setActiveCollisionObjects(link_names);
//   manager->setContactDistanceThreshold(contact_distance);
//
//   joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, &callbackJointState);
//   contact_results_pub = node->create_publisher<tesseract_msgs::msg::ContactResultVector>("contact_results", 1);
//   modify_env_service =
//       node->create_service<tesseract_msgs::srv::ModifyEnvironment>("modify_environment", &callbackModifyTesseractEnv);
//
//   if (publish_environment)
//     environment_pub = node->create_publisher<tesseract_msgs::msg::TesseractState>("tesseract", 100);
//
//   compute_contact_results = node->create_service<tesseract_msgs::srv::ComputeContactResultVector>(
//       "compute_contact_results", &callbackComputeContactResultVector);
//
//   environment_diff_sub =
//       node->create_subscription<tesseract_msgs::msg::TesseractState>("tesseract_diff", 100, &callbackTesseractEnvDiff);
//
//   RCLCPP_INFO(node->get_logger(), "Contact Monitor Running!");
//
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//
//   return 0;
// }
