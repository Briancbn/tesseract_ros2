/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <tesseract_common/macros.h>
// TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
//#include <ros/console.h>
//#include <dynamic_reconfigure/server.h>
// #include <shared_mutex>
// #include <memory>
// #include <numeric>
// #include <iostream>

//#include <tesseract_monitoring/EnvironmentMonitorDynamicReconfigureConfig.h>
// TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_rosutils/utils.h>

// #include <tesseract_environment/kdl/kdl_env.h>
// #include <tesseract_environment/core/utils.h>
// #include <tesseract_monitoring/environment_monitor.h>
// #include <tesseract_kinematics/core/utils.h>
// #include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
// #include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
// #include <tesseract_scene_graph/utils.h>
// #include <tesseract_scene_graph/resource_locator.h>

namespace tesseract_monitoring
{
  const std::string DEFAULT_ROBOT_DESCRIPTION_PARAM =
       "robot_description"; /**< Default ROS parameter for robot description */

EnvironmentMonitor::EnvironmentMonitor(const std::string& robot_description,
                                       rclcpp::Node::SharedPtr node,
                                       std::string monitor_namespace,
                                       std::string discrete_plugin,
                                       std::string continuous_plugin)
  : node_(node)
  , clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  , monitor_namespace_(std::move(monitor_namespace))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;

  node_->declare_parameter("desc_param");

  node_->get_parameter_or<std::string>("desc_param", robot_description, DEFAULT_ROBOT_DESCRIPTION_PARAM);

  node_->declare_parameter(robot_description);
  node_->declare_parameter(robot_description + "_semantic");
  node_->declare_parameter("discrete_plugin");
  node_->declare_parameter("continuous_plugin");

  node_->get_parameter_or<std::string>("discrete_plugin", discrete_plugin_name_, "");
  node_->get_parameter_or<std::string>("continuous_plugin", continuous_plugin_name_, "");

  if (!node_->has_parameter(robot_description))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to find parameter: %s", robot_description.c_str());
    return;
  }

  if (!node_->has_parameter(robot_description + "_semantic"))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to find parameter: %s", (robot_description + "_semantic").c_str());
    return;
  }

  // node_->get_parameter(robot_description, urdf_xml_string);
  // node_->get_parameter(robot_description + "_semantic", srdf_xml_string);

  env_ = std::make_shared<tesseract_environment::Environment>();
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<tesseract_environment::OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return;

  if (!initialize())
  {
    RCLCPP_ERROR(node_->get_logger(), "EnvironmentMonitor Robot Description Constructor failed to initialize!");
  }
}

EnvironmentMonitor::EnvironmentMonitor(tesseract_environment::Environment::Ptr env,
                                       rclcpp::Node::SharedPtr node,
                                       std::string monitor_namespace,
                                       std::string discrete_plugin,
                                       std::string continuous_plugin)
  : node_(node)
  , clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  , monitor_namespace_(std::move(monitor_namespace))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
  , env_(std::move(env))
{
  if (!initialize())
  {
    RCLCPP_ERROR(node_->get_logger(), "EnvironmentMonitor Tesseract Constructor failed to initialize!");
  }
}

EnvironmentMonitor::~EnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();

  current_state_monitor_.reset();
  env_ = nullptr;

  // shutdown();
}

bool EnvironmentMonitor::waitForConnection(double timeout) const
{
  rclcpp::WallTime start_time = clock_->now();
  boost::chrono::duration<double> wall_timeout(timeout);
  bool is_connected{ false };
  rclcpp::WallRate loop_rate{std::chrono::milliseconds(20)};

  while (rclcpp::ok())
  {
    {
      auto lock = std::shared_lock(scene_update_mutex_);
      is_connected = env_->isInitialized();
    }
    if (is_connected)
      return true;

    if (wall_timeout >= std::chrono::duration<double>::zero())
    {
      rclcpp::WallTime current_time = clock_->now();
      if ((current_time - start_time) >= wall_timeout)
        return false;
    }
    loop_rate.sleep();
  }

  return false;
}

// void EnvironmentMonitor::shutdown()
// {
//   monitored_environment_subscriber_.shutdown();
//   get_monitored_environment_changes_client_.shutdown();
//   get_monitored_environment_information_client_.shutdown();
//   modify_monitored_environment_client_.shutdown();
//   modify_environment_server_.shutdown();
//   get_environment_changes_server_.shutdown();
//   get_environment_information_server_.shutdown();
//   save_scene_graph_server_.shutdown();
// }

bool EnvironmentMonitor::initialize()
{
  enforce_next_state_update_ = false;

  if (monitor_namespace_.empty())
    RCLCPP_FATAL(node_->get_logger(), "The monitor namespace cannot be empty!");

  if (!env_->isInitialized())
  {
    RCLCPP_FATAL(node_->get_logger(), "Faild to initalize environment monitor, the tesseract is uninitialized!");
    return false;
  }

  try
  {
    // BUG: ROS 2 version of pluginlib ClassLoader can't find non-Ament packages.
    // Workaround is to manually add the path to tesseract_collision install dir to AMENT_PREFIX_PATH env variable.
    discrete_manager_loader_.reset(new DiscreteContactManagerPluginLoader("tesseract_collision",
                                                                          "tesseract_collision::"
                                                                          "DiscreteContactManager"));
    for (auto plugin : discrete_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::DiscreteContactManager::Ptr {
        return discrete_manager_loader_->createUniqueInstance(plugin);
      };
      env_->registerDiscreteContactManager(discrete_manager_loader_->getClassType(plugin), fn);

      RCLCPP_INFO(node_->get_logger(),
                  "Discrete Contact Monitor Registered: %s",
                  discrete_manager_loader_->getClassType(plugin).c_str());
    }

    // The tesseract sets a default so it is ok if one is not provided here.
    if (!discrete_plugin_name_.empty())
    {
      if (!discrete_manager_loader_->isClassAvailable(discrete_plugin_name_))
      {
        std::string msg = "\nFailed to set default discrete contact checker plugin: ";
        msg += discrete_plugin_name_ + '\n';
        msg += "  Available Plugins:\n";

        auto available_plugins = discrete_manager_loader_->getDeclaredClasses();
        for (const auto& plugin : available_plugins)
          msg += "    " + plugin + '\n';

        RCLCPP_ERROR(node_->get_logger(),
                    "%s",
                    msg.c_str());
      }
      else
      {
        env_->setActiveDiscreteContactManager(discrete_plugin_name_);
      }
    }

    continuous_manager_loader_.reset(new ContinuousContactManagerPluginLoader("tesseract_collision",
                                                                              "tesseract_collision::"
                                                                              "ContinuousContactManager"));
    for (auto plugin : continuous_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::ContinuousContactManager::Ptr {
        return continuous_manager_loader_->createUniqueInstance(plugin);
      };
      env_->registerContinuousContactManager(continuous_manager_loader_->getClassType(plugin), fn);

      RCLCPP_INFO(node_->get_logger(),
                  "Continuous Contact Monitor Registered: %s",
                  continuous_manager_loader_->getClassType(plugin).c_str());
    }

    if (!continuous_plugin_name_.empty())
    {
      if (!continuous_manager_loader_->isClassAvailable(continuous_plugin_name_))
      {
        std::string msg = "\nFailed to set default continuous contact checker plugin: ";
        msg += continuous_plugin_name_ + '\n';
        msg += "  Available Plugins:\n";

        auto available_plugins = continuous_manager_loader_->getDeclaredClasses();
        for (const auto& plugin : available_plugins)
          msg += "    " + plugin + '\n';

        RCLCPP_ERROR(node_->get_logger(),
                    "%s",
                    msg.c_str());
      }
      else
      {
        env_->setActiveContinuousContactManager(continuous_plugin_name_);
      }
    }
  }
  catch (int& /*e*/)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load tesseract contact managers plugin");
    env_.reset();
  }

  publish_environment_frequency_ = 30.0;

  last_update_time_ = last_robot_motion_time_ = clock_->now();
  last_robot_state_update_wall_time_ = clock->now();
  dt_state_update_ = std::chrono::duration<double>(0.1);

  state_update_pending_ = false;
  state_update_timer_ = node_->create_wall_timer(dt_state_update_,
                                                std::bind(&EnvironmentMonitor::stateUpdateTimerCallback, this),
                                                callback_group_);
  // Shutdown current services
  // modify_environment_server_.shutdown();
  // get_environment_changes_server_.shutdown();
  // get_environment_information_server_.shutdown();
  // save_scene_graph_server_.shutdown();

  // Create new service
  // std::string modify_environment_server = R"(/)" + monitor_namespace_ + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  // std::string get_environment_changes_server = R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  // std::string get_environment_information_server =
  //     R"(/)" + monitor_namespace_ + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;
  // std::string save_scene_graph_server = R"(/)" + monitor_namespace_ + DEFAULT_SAVE_SCENE_GRAPH_SERVICE;
  //
  // modify_environment_server_ =
  //     root_nh_.advertiseService(modify_environment_server, &EnvironmentMonitor::modifyEnvironmentCallback, this);
  //
  // get_environment_changes_server_ = root_nh_.advertiseService(
  //     get_environment_changes_server, &EnvironmentMonitor::getEnvironmentChangesCallback, this);
  //
  // get_environment_information_server_ = root_nh_.advertiseService(
  //     get_environment_information_server, &EnvironmentMonitor::getEnvironmentInformationCallback, this);
  //
  // save_scene_graph_server_ =
  //     root_nh_.advertiseService(save_scene_graph_server, &EnvironmentMonitor::saveSceneGraphCallback, this);

  modify_environment_server_ = node_->create_service<tesseract_msgs::srv::ModifyEnvironment>(
      DEFAULT_MODIFY_ENVIRONMENT_SERVICE,
      std::bind(&EnvironmentMonitor::modifyEnvironmentCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

  get_environment_changes_server_ = node_->create_service<tesseract_msgs::srv::GetEnvironmentChanges>(
      DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE,
      std::bind(&EnvironmentMonitor::getEnvironmentChangesCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

  get_environment_information_server_ = node_->create_service<tesseract_msgs::srv::GetEnvironmentInformation>(
      DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE,
      std::bind(
          &EnvironmentMonitor::getEnvironmentInformationCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

  save_scene_graph_server_ = node_->create_service<tesseract_msgs::srv::SaveSceneGraph>(
      DEFAULT_SAVE_SCENE_GRAPH_SERVICE,
      std::bind(&EnvironmentMonitor::saveSceneGraphCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

  return true;
}

const std::string& EnvironmentMonitor::getName() const { return monitor_namespace_; }

bool EnvironmentMonitor::applyCommand(const tesseract_environment::Command::ConstPtr& command)
{
  bool result = false;
  {
    auto lock = lockEnvironmentWrite();
    result = env_->applyCommand(command);
  }
  triggerEnvironmentUpdateEvent();
  return result;
}

bool EnvironmentMonitor::applyCommands(const tesseract_environment::Commands& commands)
{
  bool result = false;
  {
    auto lock = lockEnvironmentWrite();
    result = env_->applyCommands(commands);
  }
  triggerEnvironmentUpdateEvent();
  return result;
}

tesseract_scene_graph::SceneGraph::ConstPtr EnvironmentMonitor::getSceneGraph() const { return env_->getSceneGraph(); }

const tesseract_srdf::KinematicsInformation& EnvironmentMonitor::getKinematicsInformation() const
{
  return env_->getManipulatorManager()->getKinematicsInformation();
}

tesseract_environment::Environment::Ptr EnvironmentMonitor::getEnvironment() { return env_; }

tesseract_environment::Environment::ConstPtr EnvironmentMonitor::getEnvironment() const { return env_; }

void EnvironmentMonitor::stopPublishingEnvironment()
{
  if (publish_environment_)
  {
    std::unique_ptr<std::thread> copy;
    copy.swap(publish_environment_);
    new_environment_update_condition_.notify_all();
    copy->join();
    stopPublishingEnvironment();
    environment_publisher_.reset();  // TODO: right way to do this?
    RCLCPP_INFO(node_->get_logger(),
                "Stopped publishing maintained environment.");
  }
}

void EnvironmentMonitor::startPublishingEnvironment()
{
  if (!publish_environment_ && env_->isInitialized())
  {
    std::string environment_topic = R"(/)" + monitor_namespace_ + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
    environment_publisher_ = node_->create_publisher<tesseract_msgs::msg::TesseractState>(environment_topic, 100);
    RCLCPP_INFO(node_->get_logger(), "Publishing maintained environment on '%s'", environment_topic.c_str());
    publish_environment_ =
        std::make_unique<std::thread>(std::bind(&EnvironmentMonitor::environmentPublishingThread, this));
  }
}

double EnvironmentMonitor::getEnvironmentPublishingFrequency() const { return publish_environment_frequency_; }

void EnvironmentMonitor::environmentPublishingThread()
{
  RCLCPP_DEBUG(node_->get_logger(), "Started environment state publishing thread ...");

  // publish the full planning scene
  tesseract_msgs::msg::EnvironmentState start_msg;
  tesseract_rosutils::toMsg(start_msg, *(env_));

  environment_publisher_->publish(start_msg);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.5)));
  environment_publisher_->publish(start_msg);

  RCLCPP_DEBUG(node_->get_logger(), "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());

  do
  {
    tesseract_msgs::msg::EnvironmentState msg;
    bool publish_msg = false;
    rclcpp::Rate rate(publish_environment_frequency_);
    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      tesseract_rosutils::toMsg(msg, *env_);

      // also publish timestamp of this robot_state
      msg.joint_state.header.stamp = last_robot_motion_time_;
      publish_msg = true;
    }

    if (publish_msg)
    {
      rate.reset();
      environment_publisher_->publish(msg);
      rate.sleep();
    }
  } while (publish_environment_);
}

// void EnvironmentMonitor::stopMonitoringEnvironment()
// {
//   get_monitored_environment_changes_client_.shutdown();
//   modify_monitored_environment_client_.shutdown();
//   get_monitored_environment_information_client_.shutdown();
//   monitored_environment_subscriber_.shutdown();
//   ROS_INFO_NAMED(monitor_namespace_, "Stopped monitoring environment.");
// }

CurrentStateMonitor::ConstPtr EnvironmentMonitor::getStateMonitor() const { return current_state_monitor_; }

CurrentStateMonitor::Ptr EnvironmentMonitor::getStateMonitor() { return current_state_monitor_; }

void EnvironmentMonitor::startMonitoringEnvironment(const std::string& monitored_namespace,
                                                    MonitoredEnvironmentMode mode)
{
  monitored_environment_mode_ = mode;
  std::string monitored_environment_topic = R"(/)" + monitored_namespace + DEFAULT_PUBLISH_ENVIRONMENT_TOPIC;
  std::string monitored_environment_changes_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;
  std::string monitored_environment_modify_service = R"(/)" + monitored_namespace + DEFAULT_MODIFY_ENVIRONMENT_SERVICE;
  std::string monitored_environment_information_service =
      R"(/)" + monitored_namespace + DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;

  // stopMonitoringEnvironment();

  get_monitored_environment_changes_client_ =
      node_->create_client<tesseract_msgs::srv::GetEnvironmentChanges>(monitored_environment_changes_service);
  modify_monitored_environment_client_ =
      node_->create_client<tesseract_msgs::srv::ModifyEnvironment>(monitored_environment_modify_service);
  get_monitored_environment_information_client_ =
      node_->create_client<tesseract_msgs::srv::GetEnvironmentInformation>(monitored_environment_information_service);

  monitored_environment_subscriber_ =
      node_->create_subscription<tesseract_msgs::msg::Environment>(monitored_environment_topic, 1000, std::bind(&EnvironmentMonitor::newEnvironmentStateCallback, this, _1));
  RCLCPP_INFO(node_->get_logger(), "Monitoring external environment on '%s'", monitored_environment_topic.c_str());
}

void EnvironmentMonitor::getStateMonitoredTopics(std::vector<std::string>& topics) const
{
  topics.clear();
  if (current_state_monitor_)
  {
    const std::string& t = current_state_monitor_->getMonitoredTopic();
    if (!t.empty())
      topics.push_back(t);
  }
}

double EnvironmentMonitor::getStateUpdateFrequency() const
{
  if (!dt_state_update_.isZero())
    return 1.0 / dt_state_update_.toSec();

  return 0.0;
}

void EnvironmentMonitor::triggerEnvironmentUpdateEvent()
{
  // do not modify update functions while we are calling them
  std::scoped_lock<std::recursive_mutex> lock(update_lock_);

  for (auto& update_callback : update_callbacks_)
    update_callback();

  new_environment_update_condition_.notify_all();
}

void EnvironmentMonitor::newEnvironmentStateCallback(const tesseract_msgs::EnvironmentStateConstPtr& env)
{
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    last_update_time_ = clock_->now();

    if (!env_->isInitialized())
    {
      auto res = std::make_shared<tesseract_msgs::srv::GetEnvironmentInformation>();
      res->request.flags = tesseract_msgs::srv::GetEnvironmentInformation::Request::COMMAND_HISTORY |
                          tesseract_msgs::srv::GetEnvironmentInformation::Request::KINEMATICS_INFORMATION;

      auto result_future = get_monitored_environment_information_client_->async_send_request(res);
      if (rclcpp::spin_until_future_complete(node, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[%s]: newEnvironmentStateCallback: Failed to get monitor environment information!",
                     monitor_namespace_.c_str());
        return;
      }
      auto result = result_future.get();
      tesseract_environment::Commands commands;
      try
      {
        commands = tesseract_rosutils::fromMsg(result->response.command_history);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR(node_->get_logger(),
                        "[%s]: newEnvironmentStateCallback: Failed to convert command history message, %s!",
                        monitor_namespace_.c_str(),
                        e.what());
        return;
      }

      if (!env_->init<tesseract_environment::OFKTStateSolver>(commands))
      {
        RCLCPP_ERROR(node_->get_logger(),
                    "[%s]: newEnvironmentStateCallback: Failed to initialize environment!"),
                    monitor_namespace_.c_str()
                    );
        return;
      }

      if (!initialize())
      {
        RCLCPP_WARN(node_->get_logger(),"newEnvironmentStateCallback: EnvironmentMonitor Failed to initialize!");
      }
    }
    else
    {
      // If the monitored environment has changed then request the changes and apply
      if (static_cast<int>(env->revision) > env_->getRevision())
      {
        auto res = std::make_shared<tesseract_msgs::srv::GetEnvironmentChanges>();
        res->request.revision = static_cast<unsigned long>(env_->getRevision());
        auto result_future = get_monitored_environment_information_client_->async_send_request(res);
        if (rclcpp::spin_until_future_complete(node, result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
          if (!tesseract_rosutils::processMsg(*env_, result_future.get()->response.commands))
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "[%s]: newEnvironmentStateCallback: Failed to apply monitored environments changes.",
                         monitor_namespace_.c_str()
                        );
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "[%s]: newEnvironmentStateCallback: Failed to get monitored environments changes.",
                       monitor_namespace_.c_str()
                      );
        }
      }
      else if (static_cast<int>(env->revision) < env_->getRevision())
      {
        if (monitored_environment_mode_ == MonitoredEnvironmentMode::DEFAULT)
        {
          // If the monitored environment has a lower revision it is reset and additional changes are requested and
          // applied.
          if (env_->reset())
          {
            if (static_cast<int>(env->revision) > env_->getRevision())
            {
              auto res = std::make_shared<tesseract_msgs::srv::GetEnvironmentChanges>();
              res->request.revision = static_cast<unsigned long>(env_->getRevision());
              auto result_future = get_monitored_environment_information_client_->async_send_request(res);
              if (rclcpp::spin_until_future_complete(node, result_future) ==
                  rclcpp::FutureReturnCode::SUCCESS)
              {
                if (!tesseract_rosutils::processMsg(*env_, result_future.get().response.commands))
                {
                  RCLCPP_ERROR(node_->get_logger(),
                              "newEnvironmentStateCallback: Failed to apply monitored environments changes.",
                              monitor_namespace_.c_str());
                }
              }
              else
              {
                RCLCPP_ERROR(node_->get_logger(),
                            "[%s]: newEnvironmentStateCallback: Failed to get monitored environments changes.",
                            monitored_namespace_.c_str()
                            );
              }
            }
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(),
                        "[%s]: newEnvironmentStateCallback: Failed to reset the tesseract object!",
                        monitor_namespace_.c_str()
                        );
          }
        }
        else if (monitored_environment_mode_ == MonitoredEnvironmentMode::SYNCHRONIZED)
        {
          // If this has been modified it will push the changes to the monitored environment to keep them in sync
          auto res = std::make_shared<tesseract_msgs::srv::ModifyEnvironment>();
          res->request.id = env_->getName();
          res->request.revision = env->revision;
          if (tesseract_rosutils::toMsg(res->request.commands, env_->getCommandHistory(), env->revision))
          {
            bool status = modify_monitored_environment_client_.call(res);
            if (rclcpp::spin_until_future_complete(node, result_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
              RCLCPP_ERROR(node_->get_logger(),
                          "[%s]: newEnvironmentStateCallback: Failed to update monitored environment!",
                          monitored_namespace_.c_str()
                          );
            }
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(),
                        "[%s]: newEnvironmentStateCallback: Failed to convert latest changes to message and update monitored environment!",
                        monitored_namespace_.c_str()
                        );
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),
                      "[%s]: newEnvironmentStateCallback: Unsupporte MonitoredEnvironmentMode!",
                      monitored_namespace_.c_str()
                      );
        }
      }
    }

    if (!tesseract_rosutils::isMsgEmpty(env->joint_state))
    {
      if (last_robot_motion_time_ != env->joint_state.header.stamp)
      {
        tesseract_rosutils::processMsg(env_, env->joint_state);
        last_robot_motion_time_ = env->joint_state.header.stamp;
      }
    }

    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "[ " << monitored_namespace << " ]: environment update " << fmod(last_update_time_.toSec(), 10.)
                                                 << " robot stamp: " << fmod(last_robot_motion_time_.toSec(), 10.));
  }
  triggerEnvironmentUpdateEvent();
}

bool EnvironmentMonitor::applyEnvironmentCommandsMessage(
    const std::string& id,
    int revision,
    const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands)
{
  if (!env_ || id != env_->getName() || revision != env_->getRevision())
    return false;

  bool result = true;

  // Update joint state is not a tracked command so need to filter them out.
  std::vector<tesseract_msgs::msg::EnvironmentCommand> filtered_commands;
  std::vector<tesseract_msgs::msg::EnvironmentCommand> update_joint_state_commands;
  for (const auto& cmd : commands)
  {
    if (cmd.command == tesseract_msgs::msg::EnvironmentCommand::UPDATE_JOINT_STATE)
      update_joint_state_commands.push_back(cmd);
    else
      filtered_commands.push_back(cmd);
  }

  std::string old_scene_name;
  {
    std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
    if (!filtered_commands.empty())
      result = tesseract_rosutils::processMsg(*env_, filtered_commands);

    if (result)
    {
      for (const auto& cmd : update_joint_state_commands)
      {
        if (tesseract_rosutils::processMsg(env_, cmd.joint_state))
        {
          last_robot_motion_time_ = rclcpp::Time::now();
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(),"Failed to apply UPDATE_JOINT_STATE command!");
          result = false;
        }
      }
    }
  }

  triggerEnvironmentUpdateEvent();
  return result;
}

bool EnvironmentMonitor::saveSceneGraphCallback(tesseract_msgs::srv::SaveSceneGraph::Request& req,
                                                tesseract_msgs::srv::SaveSceneGraph::Response& res)
{
  res.success = !(env_ == nullptr);
  env_->getSceneGraph()->saveDOT(req.filepath);
  res.id = env_->getName();
  res.revision = static_cast<unsigned long>(env_->getRevision());

  return true;
}

bool EnvironmentMonitor::waitForCurrentState(const rclcpp::Time& t, double wait_time)
{
  if (t.isZero())
    return false;
  rclcpp::Time start = clock_->now();
  boost::chrono::duration<double> timeout(wait_time);

  RCLCPP_DEBUG(node_->get_logger(),
              "[%s]: sync robot state to: %.3fs",
              monitored_namespace_.c_str(),
              fmod(t.toSec(), 10.)
              );

  if (current_state_monitor_)
  {
    // Wait for next robot update in state monitor. Those updates are only
    // passed to PSM when robot actually moved!
    enforce_next_state_update_ = true;  // enforce potential updates to be directly applied
    bool success = current_state_monitor_->waitForCurrentState(t, wait_time);
    enforce_next_state_update_ = false;  // back to normal throttling behavior,
                                         // not applying incoming updates
                                         // immediately

    /* If the robot doesn't move, we will never receive an update from CSM in
       planning scene.
       As we ensured that an update, if it is triggered by CSM, is directly
       passed to the scene,
       we can early return true here (if we successfully received a CSM update).
       Otherwise return false. */
    if (success)
      return true;

    RCLCPP_WARN(node_->get_logger(),
                "Failed to fetch current robot state.");
    return false;
  }

  // Sometimes there is no state monitor. In this case state updates are
  // received as part of scene updates only.
  // However, scene updates are only published if the robot actually moves.
  // Hence we need a timeout!
  // As publishing planning scene updates is throttled (2Hz by default), a 1s
  // timeout is a suitable default.
  std::shared_lock<std::shared_mutex> lock(scene_update_mutex_);
  rclcpp::Time prev_robot_motion_time = last_robot_motion_time_;
  while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
         timeout > boost::chrono::duration<double>::zero())
  {
    RCLCPP_DEBUG(node_->get_logger(),
                "last robot motion: %f ago",
                to_chrono<std::chrono::duration<double>>().count());
    new_environment_update_condition_.wait_for(lock, std::chrono::duration<double>(timeout.count()));
    timeout = boost::chrono::duration<double>(
        timeout.count() -
        (clock_->now() - start).to_chrono<std::chrono::duration<double>>().count());  // compute remaining wait_time  //
                                                                                      // TODO: this probably introduces
                                                                                      // some weird error
  }
  bool success = last_robot_motion_time_ >= t;
  // suppress warning if we received an update at all
  if (!success && prev_robot_motion_time != last_robot_motion_time_)
    RCLCPP_WARN(node_->get_logger(),
               "Maybe failed to update robot state, time diff: %.3fs",
               (t - last_robot_motion_time_).seconds());

  // RCLCPP_DEBUG_STREAM_NAMED(node_->get_logger(),
  //                     "[ " << monitor_namespace_ << " ]: sync done: robot motion: " << (t - last_robot_motion_time_).toSec()
  //                                                    << " scene update: " << (t - last_update_time_).toSec());
  //                                                     TODO: implement
  return success;
}

std::shared_lock<std::shared_mutex> EnvironmentMonitor::lockEnvironmentRead()
{
  return std::shared_lock(scene_update_mutex_);
}
std::unique_lock<std::shared_mutex> EnvironmentMonitor::lockEnvironmentWrite()
{
  return std::unique_lock(scene_update_mutex_);
}

void EnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic, bool publish_tf)
{
  stopStateMonitor();
  if (env_)
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(
        new CurrentStateMonitor(env_, node_));

    current_state_monitor_->addUpdateCallback(boost::bind(&EnvironmentMonitor::onJointStateUpdate, this, _1));
    current_state_monitor_->startStateMonitor(joint_states_topic, publish_tf);

    {
      std::scoped_lock lock(state_pending_mutex_);
      if (dt_state_update_ != std::chrono::duration<double>::zero())
        state_update_timer_->reset();  // BUG was .start(), does ->reset() do the same thing?
    }
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(),
                "Cannot monitor robot state because planning scene is not configured");
  }
}

void EnvironmentMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  state_update_timer_->cancel();  // BUG was .stop(), changed to ->cancel()
  {
    std::scoped_lock lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void EnvironmentMonitor::onJointStateUpdate(const sensor_msgs::JointStateConstPtr& /* joint_state */)
{
  const rclcpp::Time& n = clock_->now();
  rclcpp::Duration dt = n - last_robot_state_update_wall_time_;

  bool update = enforce_next_state_update_;
  {
    std::scoped_lock lock(state_pending_mutex_);

    if (dt < dt_state_update_ && !update)
    {
      state_update_pending_ = true;
    }
    else
    {
      state_update_pending_ = false;
      last_robot_state_update_wall_time_ = n;
      update = true;
    }
  }
  // run the state update with state_pending_mutex_ unlocked
  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::updateJointStateTimerCallback( /* const rclcpp::WallTimerEvent& event*/)
{
  if (state_update_pending_)
  {
    bool update = false;

    const rclcpp::Time& n = clock_->now();
    rclcpp::Duration dt = n - last_robot_state_update_wall_time_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      std::scoped_lock lock(state_pending_mutex_);
      if (state_update_pending_ && dt >= dt_state_update_)
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = clock_->now();
        update = true;
        // RCLCPP_DEBUG_STREAM(node_->get_logger(),
        //                     "[ " << monitor_namespace_ << " ]: performPendingStateUpdate: " << fmod(last_robot_state_update_wall_time_.toSec(), 10));
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
    {
      updateEnvironmentWithCurrentState();
      // RCLCPP_DEBUG(node_->get_logger(),
      //             "[%s]: performPendingStateUpdate done",
      //             monitored_namespace_.c_str()
      //             );
    }
  }
}

void EnvironmentMonitor::setStateUpdateFrequency(double hz)
{
  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    std::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = std::chrono::duration<double>(1.0 / hz);
    // state_update_timer_.setPeriod(dt_state_update_);
    // state_update_timer_.start();
    state_update_timer_.reset();
    state_update_timer_ = node_->create_wall_timer(std::chrono::duration<double>(dt_state_update_),
                                                   std::bind(&EnvironmentMonitor::stateUpdateTimerCallback, this));
  }
  else
  {
    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    // state_update_timer_.stop();
    // std::scoped_lock lock(state_pending_mutex_);
    // dt_state_update_ = rclcpp::WallDuration(0, 0);
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = std::chrono::duration<double>(0.0);
    state_update_timer_.reset();
    state_update_timer_ = node_->create_wall_timer(std::chrono::duration<double>(dt_state_update_),
                                                   std::bind(&EnvironmentMonitor::stateUpdateTimerCallback, this));
    if (state_update_pending_)
      update = true;
  }
  // RCLCPP_INFO(node_->get_logger(),
  //             "[%s]: Updating internal planning scene state at most every %lf seconds",
  //             monitored_namespace_.c_str(),
  //             dt_state_update_.toSec()
  //             );
  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::updateEnvironmentWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (clock_->now() - current_state_monitor_->getMonitorStartTime()).seconds() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      RCLCPP_WARN_ONCE(
          node_->get_logger(), "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }

    {
      std::unique_lock<std::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
      RCLCPP_DEBUG(node_->get_logger(), "robot state update %f ", fmod(last_robot_motion_time_.seconds(), 10.));

      env_->setState(current_state_monitor_->getCurrentState()->joints);
    }
    triggerEnvironmentUpdateEvent();
  }
  else
    RCLCPP_ERROR_ONCE(node_->get_logger(), "State monitor is not active. Unable to set the planning scene state");

}

void EnvironmentMonitor::addUpdateCallback(const std::function<void()>& fn)
{
  std::scoped_lock lock(update_lock_);
  if (fn)
    update_callbacks_.push_back(fn);
}

void EnvironmentMonitor::clearUpdateCallbacks()
{
  std::scoped_lock lock(update_lock_);
  update_callbacks_.clear();
}

void EnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
{
  publish_environment_frequency_ = hz;
  RCLCPP_DEBUG(node_->get_logger(),
              "[%s]: Maximum frquency for publishing an environment is now %lf Hz",
              monitor_namespace_.c_str(),
              publish_environment_frequency_);
}

bool EnvironmentMonitor::modifyEnvironmentCallback(
    const std::shared_ptr<tesseract_msgs::srv::ModifyEnvironment::Request> req,
    std::shared_ptr<tesseract_msgs::srv::ModifyEnvironment::Response> res)
{
  if (req->append)
    res->success = applyEnvironmentCommandsMessage(req->id, env_->getRevision(), req->commands);
  else
    res->success = applyEnvironmentCommandsMessage(req->id, static_cast<int>(req->revision), req->commands);

  res->revision = static_cast<unsigned long>(env_->getRevision());
  return res.success;
}

bool EnvironmentMonitor::getEnvironmentChangesCallback(
    const std::shared_ptr<tesseract_msgs::srv::GetEnvironmentChanges::Request> req,
    std::shared_ptr<tesseract_msgs::srv::GetEnvironmentChanges::Response> res)
{
  auto lock_read = lockEnvironmentRead();

  if (static_cast<int>(req->revision) > env_->getRevision())
  {
    res->success = false;
    return false;
  }

  res->id = env_->getName();
  res->revision = static_cast<unsigned long>(env_->getRevision());
  if (!tesseract_rosutils::toMsg(res->commands, env_->getCommandHistory(), req->revision))
  {
    res->success = false;
    return false;
  }

  res->success = true;
  return res->success;
}

bool EnvironmentMonitor::getEnvironmentInformationCallback(
    const std::shared_ptr<tesseract_msgs::srv::GetEnvironmentInformation::Request> req,
    std::shared_ptr<tesseract_msgs::srv::GetEnvironmentInformation::Response> res)
{
  auto lock_read = lockEnvironmentRead();

  if (!env_->isInitialized())
  {
    res->success = false;
    return false;
  }

  res->id = env_->getName();
  res->revision = static_cast<unsigned long>(env_->getRevision());

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::COMMAND_HISTORY)
  {
    if (!tesseract_rosutils::toMsg(res->command_history, env_->getCommandHistory(), 0))
    {
      res->success = false;
      return false;
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_LIST)
  {
    for (const auto& link : env_->getSceneGraph()->getLinks())
    {
      tesseract_msgs::msg::Link msg;
      if (!tesseract_rosutils::toMsg(msg, *link))
      {
        res->success = false;
        return false;
      }
      res->links.push_back(msg);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_LIST)
  {
    for (const auto& joint : env_->getSceneGraph()->getJoints())
    {
      tesseract_msgs::msg::Joint msg;
      if (!tesseract_rosutils::toMsg(msg, *joint))
      {
        res->success = false;
        return false;
      }
      res->joints.push_back(msg);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_NAMES)
  {
    for (const auto& link : env_->getLinkNames())
    {
      res->link_names.push_back(link);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_NAMES)
  {
    for (const auto& joint : env_->getJointNames())
    {
      res->joint_names.push_back(joint);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ACTIVE_LINK_NAMES)
  {
    for (const auto& link : env_->getActiveLinkNames())
    {
      res->active_link_names.push_back(link);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ACTIVE_JOINT_NAMES)
  {
    for (const auto& joint : env_->getActiveJointNames())
    {
      res->active_joint_names.push_back(joint);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_TRANSFORMS)
  {
    for (const auto& link_pair : env_->getCurrentState()->link_transforms)
    {
      res->link_transforms.names.push_back(link_pair.first);
      geometry_msgs::msg::Pose pose = tf2::toMsg(link_pair.second);
      res->link_transforms.transforms.push_back(pose);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_TRANSFORMS)
  {
    for (const auto& joint_pair : env_->getCurrentState()->joint_transforms)
    {
      res->joint_transforms.names.push_back(joint_pair.first);
      geometry_msgs::msg::Pose pose;
      tesseract_rosutils::toMsg(pose, joint_pair.second);
      res->joint_transforms.transforms.push_back(pose);
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ALLOWED_COLLISION_MATRIX)
  {
    if (!tesseract_rosutils::toMsg(res->allowed_collision_matrix, *env_->getAllowedCollisionMatrix()))
    {
      res->success = false;
      return false;
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::KINEMATICS_INFORMATION)
  {
    auto manipulator_manager = env_->getManipulatorManager();
    if (!tesseract_rosutils::toMsg(res.kinematics_information, manipulator_manager->getKinematicsInformation()))
    {
      res->success = false;
      return false;
    }
  }

  if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_STATES)
  {
    if (!tesseract_rosutils::toMsg(res->joint_states, env_->getCurrentState()->joints))
    {
      res->success = false;
      return false;
    }
  }

  res->success = true;
  return true;
}

}  // namespace tesseract_monitoring


//
// // TODO: Dynamic reconfigure deprecated in ROS 2. Use parameter updates instead.
// // class DynamicReconfigureImpl
// //{
// // public:
// //  DynamicReconfigureImpl(tesseract_monitoring::EnvironmentMonitor* owner)
// //    : owner_(owner), dynamic_reconfigure_server_(ros::NodeHandle(decideNamespace(owner->getName())))
// //  {
// //    dynamic_reconfigure_server_.setCallback(
// //        boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
// //  }
//
// // private:
// //  // make sure we do not advertise the same service multiple times, in case we
// //  // use multiple PlanningSceneMonitor
// //  // instances in a process
// //  static std::string decideNamespace(const std::string& name)
// //  {
// //    std::string ns = "~/" + name;
// //    std::replace(ns.begin(), ns.end(), ' ', '_');
// //    std::transform(ns.begin(), ns.end(), ns.begin(), ::tolower);
// //    if (ros::service::exists(ns + "/set_parameters", false))
// //    {
// //      unsigned int c = 1;
// //      while (ros::service::exists(ns + boost::lexical_cast<std::string>(c) + "/set_parameters", false))
// //        c++;
// //      ns += boost::lexical_cast<std::string>(c);
// //    }
// //    return ns;
// //  }
//
// //  void dynamicReconfigureCallback(tesseract_monitoring::EnvironmentMonitorDynamicReconfigureConfig& config,
// //                                  uint32_t /*level*/)
// //  {
// //    using namespace tesseract_monitoring;
// //    EnvironmentMonitor::EnvironmentUpdateType event = EnvironmentMonitor::UPDATE_NONE;
// //    if (config.publish_geometry_updates)
// //      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_GEOMETRY);
// //    if (config.publish_state_updates)
// //      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_STATE);
// //    if (config.publish_transforms_updates)
// //      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_TRANSFORMS);
// //    if (config.publish_environment)
// //    {
// //      owner_->setEnvironmentPublishingFrequency(config.publish_environment_hz);
// //      owner_->startPublishingEnvironment(event);
// //    }
// //    else
// //      owner_->stopPublishingEnvironment();
// //  }
//
// //  tesseract_monitoring::EnvironmentMonitor* owner_;
// //  dynamic_reconfigure::Server<tesseract_monitoring::EnvironmentMonitorDynamicReconfigureConfig>
// //      dynamic_reconfigure_server_;
// //};
//
// // namespace tesseract_monitoring
// // {
// // static const std::string LOGNAME = "environment_monitor";
// // const std::string DEFAULT_ROBOT_DESCRIPTION_PARAM =
// //     "robot_description"; /**< Default ROS parameter for robot description */
// // const std::string EnvironmentMonitor::DEFAULT_JOINT_STATES_TOPIC = "joint_states";
// // const std::string EnvironmentMonitor::DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes";
// // const std::string EnvironmentMonitor::DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE = "get_tesseract_information";
// // const std::string EnvironmentMonitor::DEFAULT_MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract";
// // const std::string EnvironmentMonitor::DEFAULT_SAVE_SCENE_GRAPH_SERVICE = "save_scene_graph";
// // const std::string EnvironmentMonitor::MONITORED_ENVIRONMENT_TOPIC = "monitored_tesseract";
// //
// EnvironmentMonitor::EnvironmentMonitor(const std::string& name, rclcpp::Node::SharedPtr node)
//   : node_(node)
//   , clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
//   , monitor_name_(name)
//   , dt_state_update_(0)
//   , shape_transform_cache_lookup_wait_time_(0)
//   , callback_group_(node_->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive))
// {
//   // Initial setup
//   std::string robot_description;
//
//   node_->declare_parameter("desc_param");
//
//   node_->get_parameter_or<std::string>("desc_param", robot_description, DEFAULT_ROBOT_DESCRIPTION_PARAM);
//
//   node_->declare_parameter(robot_description);
//   node_->declare_parameter(robot_description + "_semantic");
//   node_->declare_parameter("discrete_plugin");
//   node_->declare_parameter("continuous_plugin");
//   node_->declare_parameter("joint_state_topic");
//   node_->declare_parameter("monitored_environment_topic");
//
//   node_->get_parameter_or<std::string>("discrete_plugin", discrete_plugin_name_, "");
//   node_->get_parameter_or<std::string>("continuous_plugin", continuous_plugin_name_, "");
//   node_->get_parameter_or<std::string>("joint_state_topic", joint_state_topic_, "");
//   node_->get_parameter_or<std::string>("monitored_environment_topic", monitored_environment_topic_, "");
//
//   std::string urdf_path, srdf_path;
//   if (!node_->get_parameter(robot_description, urdf_path))
//   {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to find required parameter: %s", robot_description.c_str());
//     return;
//   }
//
//   if (!node_->get_parameter(robot_description + "_semantic", srdf_path))
//   {
//     RCLCPP_ERROR(
//         node_->get_logger(), "Failed to find required parameter: %s", (robot_description + "_semantic").c_str());
//     return;
//   }
//
//   std::stringstream urdf_xml_string, srdf_xml_string;
//   std::ifstream urdf_in(urdf_path);
//   urdf_xml_string << urdf_in.rdbuf();
//   std::ifstream srdf_in(srdf_path);
//   srdf_xml_string << srdf_in.rdbuf();
//
//   tesseract_ = std::make_shared<tesseract::Tesseract>();
//   tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
//   if (!tesseract_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator))
//     return;
//
//   initialize();
// }
//
// // EnvironmentMonitor::EnvironmentMonitor(tesseract::Tesseract::Ptr tesseract,
// //                                       const std::string& name,
// //                                       const std::string& discrete_plugin,
// //                                       const std::string& continuous_plugin)
// //  : Node("environment_monitor")
// //  , monitor_name_(name)
// //  , discrete_plugin_name_(discrete_plugin)
// //  , continuous_plugin_name_(continuous_plugin)
// //  , tesseract_(std::move(tesseract))
// //  , dt_state_update_(0.0)
// //  , shape_transform_cache_lookup_wait_time_(0, 0)
// //{
// //  initialize();
// //}
//
// EnvironmentMonitor::~EnvironmentMonitor()
// {
//   stopPublishingEnvironment();
//   stopStateMonitor();
//
//   //  delete reconfigure_impl_;
//   current_state_monitor_.reset();
//   tesseract_.reset();
// }
//
// void EnvironmentMonitor::initialize()
// {
//   enforce_next_state_update_ = false;
//
//   if (monitor_name_.empty())
//     monitor_name_ = "tesseract_monitor";
//
//   if (!tesseract_->isInitialized())
//   {
//     RCLCPP_FATAL(node_->get_logger(), "Failed to initalize environment monitor");
//     return;
//   }
//   else
//   {
//     try
//     {
//       // BUG: ROS 2 version of pluginlib ClassLoader can't find non-Ament packages.
//       // Workaround is to manually add the path to tesseract_collision install dir to AMENT_PREFIX_PATH env variable.
//       discrete_manager_loader_.reset(new DiscreteContactManagerPluginLoader("tesseract_collision",
//                                                                             "tesseract_collision::"
//                                                                             "DiscreteContactManager"));
//       for (auto plugin : discrete_manager_loader_->getDeclaredClasses())
//       {
//         auto fn = [&]() -> tesseract_collision::DiscreteContactManager::Ptr {
//           return discrete_manager_loader_->createUniqueInstance(plugin);
//         };
//         tesseract_->getEnvironment()->registerDiscreteContactManager(discrete_manager_loader_->getClassType(plugin),
//                                                                      fn);
//
//         RCLCPP_INFO(node_->get_logger(),
//                     "Discrete Contact Monitor Registered: %s",
//                     discrete_manager_loader_->getClassType(plugin).c_str());
//       }
//
//       // The tesseract sets a default so it is ok if one is not provided here.
//       if (!discrete_plugin_name_.empty())
//       {
//         if (discrete_manager_loader_->isClassAvailable(discrete_plugin_name_))
//         {
//           RCLCPP_ERROR(node_->get_logger(),
//                        "Failed to set default tesseract contact checker plugin: %s.",
//                        discrete_plugin_name_.c_str());
//         }
//         else
//         {
//           tesseract_->getEnvironment()->setActiveDiscreteContactManager(discrete_plugin_name_);
//         }
//       }
//
//       continuous_manager_loader_.reset(new ContinuousContactManagerPluginLoader("tesseract_collision",
//                                                                                 "tesseract_collision::"
//                                                                                 "ContinuousContactManager"));
//       for (auto plugin : continuous_manager_loader_->getDeclaredClasses())
//       {
//         auto fn = [&]() -> tesseract_collision::ContinuousContactManager::Ptr {
//           return continuous_manager_loader_->createUniqueInstance(plugin);
//         };
//         tesseract_->getEnvironment()->registerContinuousContactManager(continuous_manager_loader_->getClassType(plugin),
//                                                                        fn);
//
//         RCLCPP_INFO(node_->get_logger(),
//                     "Continuous Contact Monitor Registered: %s",
//                     continuous_manager_loader_->getClassType(plugin).c_str());
//       }
//
//       if (!continuous_plugin_name_.empty())
//       {
//         if (continuous_manager_loader_->isClassAvailable(continuous_plugin_name_))
//         {
//           RCLCPP_ERROR(node_->get_logger(),
//                        "Failed to set default tesseract contact checker plugin: %s.",
//                        continuous_plugin_name_.c_str());
//         }
//         else
//         {
//           tesseract_->getEnvironment()->setActiveContinuousContactManager(continuous_plugin_name_);
//         }
//       }
//     }
//     catch (int& /*e*/)
//     {
//       RCLCPP_ERROR(node_->get_logger(), "Failed to load tesseract contact managers plugin");
//       tesseract_.reset();
//     }
//   }
//
//   publish_environment_frequency_ = 2.0;
//   new_environment_update_ = UPDATE_NONE;
//
//   last_update_time_ = last_robot_motion_time_ = clock_->now();
//   last_robot_state_update_wall_time_ = clock_->now();
//   dt_state_update_ = std::chrono::duration<double>(0.1);
//
//   state_update_pending_ = false;
//
//   state_update_timer_ = node_->create_wall_timer(
//       dt_state_update_, std::bind(&EnvironmentMonitor::stateUpdateTimerCallback, this), callback_group_);
//
//   //  reconfigure_impl_ = new DynamicReconfigureImpl(this);
//
//   modify_environment_server_ = node_->create_service<tesseract_msgs::srv::ModifyEnvironment>(
//       DEFAULT_MODIFY_ENVIRONMENT_SERVICE,
//       std::bind(&EnvironmentMonitor::modifyEnvironmentCallback, this, std::placeholders::_1, std::placeholders::_2),
//       rmw_qos_profile_services_default,
//       callback_group_);
//
//   get_environment_changes_server_ = node_->create_service<tesseract_msgs::srv::GetEnvironmentChanges>(
//       DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE,
//       std::bind(&EnvironmentMonitor::getEnvironmentChangesCallback, this, std::placeholders::_1, std::placeholders::_2),
//       rmw_qos_profile_services_default,
//       callback_group_);
//
//   get_environment_information_server_ = node_->create_service<tesseract_msgs::srv::GetEnvironmentInformation>(
//       DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE,
//       std::bind(
//           &EnvironmentMonitor::getEnvironmentInformationCallback, this, std::placeholders::_1, std::placeholders::_2),
//       rmw_qos_profile_services_default,
//       callback_group_);
//
//   save_scene_graph_server_ = node_->create_service<tesseract_msgs::srv::SaveSceneGraph>(
//       DEFAULT_SAVE_SCENE_GRAPH_SERVICE,
//       std::bind(&EnvironmentMonitor::saveSceneGraphCallback, this, std::placeholders::_1, std::placeholders::_2),
//       rmw_qos_profile_services_default,
//       callback_group_);
// }
//
// void EnvironmentMonitor::postInitialize()
// {
//   if (monitored_environment_topic_.empty())
//     this->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);
//   else
//     this->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT,
//                                      monitored_environment_topic_);
//
//   if (joint_state_topic_.empty())
//     this->startStateMonitor();
//   else
//     this->startStateMonitor(joint_state_topic_);
// }
//
// void EnvironmentMonitor::stopPublishingEnvironment()
// {
//   if (publish_environment_)
//   {
//     std::unique_ptr<boost::thread> copy;
//     copy.swap(publish_environment_);
//     new_environment_update_condition_.notify_all();
//     copy->join();
//     stopPublishingEnvironment();
//     environment_publisher_.reset();  // TODO: right way to do this?
//     RCLCPP_INFO(node_->get_logger(), "Stopped publishing maintained environment.");
//   }
// }
//
// void EnvironmentMonitor::startPublishingEnvironment(EnvironmentUpdateType update_type,
//                                                     const std::string& environment_topic)
// {
//   publish_update_types_ = update_type;
//   if (!publish_environment_ && tesseract_->isInitialized())
//   {
//     environment_publisher_ = node_->create_publisher<tesseract_msgs::msg::TesseractState>(environment_topic, 100);
//     RCLCPP_INFO(node_->get_logger(), "Publishing maintained environment on '%s'", environment_topic.c_str());
//     publish_environment_.reset(new boost::thread(boost::bind(&EnvironmentMonitor::environmentPublishingThread, this)));
//   }
// }
//
// void EnvironmentMonitor::environmentPublishingThread()
// {
//   RCLCPP_DEBUG(node_->get_logger(), "Started environment state publishing thread ...");
//
//   // publish the full planning scene
//   tesseract_msgs::msg::TesseractState start_msg;
//   tesseract_rosutils::toMsg(start_msg, *(tesseract_->getEnvironment()));
//
//   environment_publisher_->publish(start_msg);
//   rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.5)));
//   environment_publisher_->publish(start_msg);
//
//   RCLCPP_DEBUG(node_->get_logger(), "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());
//
//   do
//   {
//     tesseract_msgs::msg::TesseractState msg;
//     bool publish_msg = false;
//     rclcpp::Rate rate(publish_environment_frequency_);
//     {
//       std::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//       while (new_environment_update_ == UPDATE_NONE && publish_environment_)
//         new_environment_update_condition_.wait(ulock);
//       if (new_environment_update_ != UPDATE_NONE)
//       {
//         if ((publish_update_types_ & new_environment_update_) || new_environment_update_ == UPDATE_ENVIRONMENT)
//         {
//           tesseract_rosutils::toMsg(msg, *(tesseract_->getEnvironment()));
//
//           // also publish timestamp of this robot_state
//           msg.joint_state.header.stamp = last_robot_motion_time_;
//           publish_msg = true;
//         }
//         new_environment_update_ = UPDATE_NONE;
//       }
//     }
//
//     if (publish_msg)
//     {
//       rate.reset();
//       environment_publisher_->publish(msg);
//       rate.sleep();
//     }
//   } while (publish_environment_);
// }
//
// void EnvironmentMonitor::getMonitoredTopics(std::vector<std::string>& topics) const
// {
//   topics.clear();
//   if (current_state_monitor_)
//   {
//     const std::string& t = current_state_monitor_->getMonitoredTopic();
//     if (!t.empty())
//       topics.push_back(t);
//   }
// }
//
// void EnvironmentMonitor::triggerEnvironmentUpdateEvent(EnvironmentUpdateType update_type)
// {
//   // do not modify update functions while we are calling them
//   boost::recursive_mutex::scoped_lock lock(update_lock_);
//
//   for (std::size_t i = 0; i < update_callbacks_.size(); ++i)
//     update_callbacks_[i](update_type);
//   new_environment_update_ = static_cast<EnvironmentUpdateType>(new_environment_update_ | update_type);
//   new_environment_update_condition_.notify_all();
// }
//
// void EnvironmentMonitor::newStateCallback(const std::shared_ptr<tesseract_msgs::msg::TesseractState> env)
// {
//   if (!tesseract_->getEnvironment())
//     return;
//
//   EnvironmentUpdateType upd = UPDATE_ENVIRONMENT;
//   std::string old_scene_name;
//   {
//     boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//
//     last_update_time_ = clock_->now();
//     last_robot_motion_time_ = env->joint_state.header.stamp;
//     RCLCPP_DEBUG(node_->get_logger(),
//                  "environment update: %d, robot stamp: %d",
//                  fmod(last_update_time_.seconds(), 10.),
//                  fmod(last_robot_motion_time_.seconds(), 10.));
//     old_scene_name = tesseract_->getEnvironment()->getName();
//     tesseract_rosutils::processMsg(tesseract_->getEnvironment(), *env);
//   }
//
//   upd = UPDATE_NONE;
//   if (!tesseract_rosutils::isMsgEmpty(env->joint_state) || !tesseract_rosutils::isMsgEmpty(env->multi_dof_joint_state))
//     upd = static_cast<EnvironmentUpdateType>(upd | UPDATE_STATE);
//
//   triggerEnvironmentUpdateEvent(upd);
// }
//
// bool EnvironmentMonitor::applyEnvironmentCommandsMessage(
//     std::string id,
//     int revision,
//     const std::vector<tesseract_msgs::msg::EnvironmentCommand>& commands)
// {
//   if (!tesseract_->getEnvironment() || id != tesseract_->getEnvironment()->getName() ||
//       revision != tesseract_->getEnvironment()->getRevision())
//     return false;
//
//   bool result;
//
//   EnvironmentUpdateType upd = UPDATE_ENVIRONMENT;
//   std::string old_scene_name;
//   {
//     boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//     result = tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), commands);
//   }
//
//   // if we have a diff, try to more accuratelly determine the update type
//   //  if (env_msg.is_diff)
//   //  {
//   //    //    bool no_other_scene_upd = (env_msg.name.empty() || env_msg.name ==
//   //    //    old_scene_name) &&
//   //    //                               env_msg.allowed_collision_matrix.entry_names.empty();
//
//   //    // TODO: Levi Need to add back allowed collision matrix
//   //    bool no_other_scene_upd = (env_msg.name.empty() || env_msg.name == old_scene_name);
//
//   //    if (no_other_scene_upd)
//   //    {
//   //      upd = UPDATE_NONE;
//   //      if (!env_msg.attachable_objects.empty() || !env_msg.attached_bodies.empty())
//   //        upd = static_cast<EnvironmentUpdateType>(upd | UPDATE_GEOMETRY);
//
//   //      //      if (!env.fixed_frame_transforms.empty())
//   //      //        upd = (EnvironmentUpdateType)((int)upd |
//   //      //        (int)UPDATE_TRANSFORMS);
//
//   //      if (!tesseract_ros::isMsgEmpty(env_msg.multi_dof_joint_state) ||
//   //          !tesseract_ros::isMsgEmpty(env_msg.multi_dof_joint_state))
//   //        upd = static_cast<EnvironmentUpdateType>(upd | UPDATE_STATE);
//   //    }
//   //  }
//   triggerEnvironmentUpdateEvent(upd);
//   return result;
// }
//
// // void EnvironmentMonitor::newPlanningSceneWorldCallback(
// //    const moveit_msgs::PlanningSceneWorldConstPtr& world)
// //{
// //  if (scene_)
// //  {
// //    updateFrameTransforms();
// //    {
// //      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
// //      last_update_time_ = ros::Time::now();
// //      scene_->getWorldNonConst()->clearObjects();
// //      scene_->processPlanningSceneWorldMsg(*world);
// //      if (octomap_monitor_)
// //      {
// //        if (world->octomap.octomap.data.empty())
// //        {
// //          octomap_monitor_->getOcTreePtr()->lockWrite();
// //          octomap_monitor_->getOcTreePtr()->clear();
// //          octomap_monitor_->getOcTreePtr()->unlockWrite();
// //        }
// //      }
// //    }
// //    triggerSceneUpdateEvent(UPDATE_SCENE);
// //  }
// //}
//
// // void EnvironmentMonitor::collisionObjectFailTFCallback(
// //    const moveit_msgs::CollisionObjectConstPtr& obj,
// //    tf::filter_failure_reasons::FilterFailureReason reason)
// //{
// //  // if we just want to remove objects, the frame does not matter
// //  if (reason == tf::filter_failure_reasons::EmptyFrameID && obj->operation ==
// //  moveit_msgs::CollisionObject::REMOVE)
// //    collisionObjectCallback(obj);
// //}
//
// // void EnvironmentMonitor::attachableObjectCallback(const
// // tesseract_msgs::AttachableObjectConstPtr &ao_msg)
// //{
// //  if (env_)
// //  {
// //    {
// //      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
// //      last_update_time_ = ros::Time::now();
//
// //      tesseract_ros::processAttachableObjectMsg(env_, *ao_msg);
// //    }
// //    triggerEnvironmentUpdateEvent(UPDATE_GEOMETRY);
// //  }
// //}
//
// // void EnvironmentMonitor::attachedBodyInfoCallback(const
// // tesseract_msgs::AttachedBodyInfoConstPtr &ab_info_msg)
// //{
// //  if (env_)
// //  {
// //    {
// //      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
// //      last_update_time_ = ros::Time::now();
//
// //      tesseract_ros::processAttachedBodyInfoMsg(env_, *ab_info_msg);
// //    }
// //    triggerEnvironmentUpdateEvent(UPDATE_GEOMETRY);
// //  }
// //}
//
// void EnvironmentMonitor::saveSceneGraphCallback(const std::shared_ptr<tesseract_msgs::srv::SaveSceneGraph::Request> req,
//                                                 std::shared_ptr<tesseract_msgs::srv::SaveSceneGraph::Response> res)
// {
//   auto env = tesseract_->getEnvironment();
//   res->success = !(env == nullptr);
//   env->getSceneGraph()->saveDOT(req->filepath);
//   res->id = env->getName();
//   res->revision = static_cast<unsigned long>(env->getRevision());
// }
//
// bool EnvironmentMonitor::waitForCurrentState(const rclcpp::Time& t, double wait_time)
// {
//   if (t.seconds() == 0)
//     return false;
//   rclcpp::Time start = clock_->now();
//   boost::chrono::duration<double> timeout(wait_time);
//
//   RCLCPP_DEBUG(node_->get_logger(), "sync robot state to: %.3fs", fmod(t.seconds(), 10.));
//
//   if (current_state_monitor_)
//   {
//     // Wait for next robot update in state monitor. Those updates are only
//     // passed to PSM when robot actually moved!
//     enforce_next_state_update_ = true;  // enforce potential updates to be directly applied
//     bool success = current_state_monitor_->waitForCurrentState(t, wait_time);
//     enforce_next_state_update_ = false;  // back to normal throttling behavior,
//                                          // not applying incoming updates
//                                          // immediately
//
//     /* If the robot doesn't move, we will never receive an update from CSM in
//        planning scene.
//        As we ensured that an update, if it is triggered by CSM, is directly
//        passed to the scene,
//        we can early return true here (if we successfully received a CSM update).
//        Otherwise return false. */
//     if (success)
//       return true;
//
//     RCLCPP_WARN(node_->get_logger(), "Failed to fetch current robot state.");
//     return false;
//   }
//
//   // Sometimes there is no state monitor. In this case state updates are
//   // received as part of scene updates only.
//   // However, scene updates are only published if the robot actually moves.
//   // Hence we need a timeout!
//   // As publishing planning scene updates is throttled (2Hz by default), a 1s
//   // timeout is a suitable default.
//   boost::shared_lock<boost::shared_mutex> lock(scene_update_mutex_);
//   rclcpp::Time prev_robot_motion_time = last_robot_motion_time_;
//   while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
//          timeout > boost::chrono::duration<double>::zero())
//   {
//     RCLCPP_DEBUG(node_->get_logger(),
//                  "last robot motion: %f ago",
//                  (t - last_robot_motion_time_).to_chrono<std::chrono::duration<double>>().count());
//     new_environment_update_condition_.wait_for(lock, std::chrono::duration<double>(timeout.count()));
//     timeout = boost::chrono::duration<double>(
//         timeout.count() -
//         (clock_->now() - start).to_chrono<std::chrono::duration<double>>().count());  // compute remaining wait_time  //
//                                                                                       // TODO: this probably introduces
//                                                                                       // some weird error
//   }
//   bool success = last_robot_motion_time_ >= t;
//   // suppress warning if we received an update at all
//   if (!success && prev_robot_motion_time != last_robot_motion_time_)
//     RCLCPP_WARN(node_->get_logger(),
//                 "Maybe failed to update robot state, time diff: %.3fs",
//                 (t - last_robot_motion_time_).seconds());
//
//   //  ROS_DEBUG_STREAM_NAMED(LOGNAME,
//   //                         "sync done: robot motion: " << (t - last_robot_motion_time_).seconds()
//   //                                                     << " scene update: " << (t - last_update_time_).seconds());  //
//   //                                                     TODO: implement
//
//   return success;
// }
//
// void EnvironmentMonitor::lockEnvironmentRead() { scene_update_mutex_.lock_shared(); }
// void EnvironmentMonitor::unlockEnvironmentRead() { scene_update_mutex_.unlock_shared(); }
// void EnvironmentMonitor::lockEnvironmentWrite() { scene_update_mutex_.lock(); }
// void EnvironmentMonitor::unlockEnvironmentWrite() { scene_update_mutex_.unlock(); }
//
// void EnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic)
// {
//   stopStateMonitor();
//   if (tesseract_->getEnvironment())
//   {
//     if (!current_state_monitor_)
//       current_state_monitor_.reset(
//           new CurrentStateMonitor(tesseract_->getEnvironment(), tesseract_->getFwdKinematicsManager(), node_));
//
//     current_state_monitor_->addUpdateCallback(boost::bind(&EnvironmentMonitor::onStateUpdate, this, _1));
//     current_state_monitor_->startStateMonitor(joint_states_topic);
//
//     {
//       boost::mutex::scoped_lock lock(state_pending_mutex_);
//       if (dt_state_update_ != std::chrono::duration<double>::zero())
//         state_update_timer_->reset();  // BUG was .start(), does ->reset() do the same thing?
//     }
//   }
//   else
//   {
//     RCLCPP_ERROR(node_->get_logger(), "Cannot monitor robot state because planning scene is not configured");
//   }
// }
//
// void EnvironmentMonitor::stopStateMonitor()
// {
//   if (current_state_monitor_)
//     current_state_monitor_->stopStateMonitor();
//
//   // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
//   state_update_timer_->cancel();  // BUG was .stop(), changed to ->cancel()
//   {
//     boost::mutex::scoped_lock lock(state_pending_mutex_);
//     state_update_pending_ = false;
//   }
// }
//
// void EnvironmentMonitor::onStateUpdate(const sensor_msgs::msg::JointState::SharedPtr /* joint_state */)
// {
//   const rclcpp::Time& n = clock_->now();
//   rclcpp::Duration dt = n - last_robot_state_update_wall_time_;
//
//   bool update = enforce_next_state_update_;
//   {
//     boost::mutex::scoped_lock lock(state_pending_mutex_);
//
//     if (dt < dt_state_update_ && !update)
//     {
//       state_update_pending_ = true;
//     }
//     else
//     {
//       state_update_pending_ = false;
//       last_robot_state_update_wall_time_ = n;
//       update = true;
//     }
//   }
//   // run the state update with state_pending_mutex_ unlocked
//   if (update)
//     updateEnvironmentWithCurrentState();
// }
//
// void EnvironmentMonitor::stateUpdateTimerCallback()
// {
//   if (state_update_pending_)
//   {
//     bool update = false;
//
//     const rclcpp::Time& n = clock_->now();
//     rclcpp::Duration dt = n - last_robot_state_update_wall_time_;
//
//     {
//       // lock for access to dt_state_update_ and state_update_pending_
//       boost::mutex::scoped_lock lock(state_pending_mutex_);
//       if (state_update_pending_ && dt >= dt_state_update_)
//       {
//         state_update_pending_ = false;
//         last_robot_state_update_wall_time_ = clock_->now();
//         update = true;
//         //        ROS_DEBUG_STREAM_NAMED(LOGNAME,
//         //                               "performPendingStateUpdate: " <<
//         //                               fmod(last_robot_state_update_wall_time_.toSec(), 10)); // TODO: implement
//       }
//     }
//
//     // run the state update with state_pending_mutex_ unlocked
//     if (update)
//     {
//       updateEnvironmentWithCurrentState();
//       //      ROS_DEBUG_NAMED(LOGNAME, "performPendingStateUpdate done"); // TODO: implement
//     }
//   }
// }
//
// void EnvironmentMonitor::setStateUpdateFrequency(double hz)
// {
//   bool update = false;
//   if (hz > std::numeric_limits<double>::epsilon())
//   {
//     boost::mutex::scoped_lock lock(state_pending_mutex_);
//     dt_state_update_ = std::chrono::duration<double>(1.0 / hz);
//     state_update_timer_.reset();
//     state_update_timer_ = node_->create_wall_timer(std::chrono::duration<double>(dt_state_update_),
//                                                    std::bind(&EnvironmentMonitor::stateUpdateTimerCallback, this));
//   }
//   else
//   {
//     // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
//     //    state_update_timer_.stop();
//     boost::mutex::scoped_lock lock(state_pending_mutex_);
//     dt_state_update_ = std::chrono::duration<double>(0.0);
//     state_update_timer_.reset();
//     state_update_timer_ = node_->create_wall_timer(std::chrono::duration<double>(dt_state_update_),
//                                                    std::bind(&EnvironmentMonitor::stateUpdateTimerCallback, this));
//     if (state_update_pending_)
//       update = true;
//   }
//   //  ROS_INFO_NAMED(LOGNAME, "Updating internal planning scene state at most every %lf seconds",
//   //  dt_state_update_.seconds());
//
//   if (update)
//     updateEnvironmentWithCurrentState();
// }
//
// void EnvironmentMonitor::updateEnvironmentWithCurrentState()
// {
//   if (current_state_monitor_)
//   {
//     std::vector<std::string> missing;
//     if (!current_state_monitor_->haveCompleteState(missing) &&
//         (clock_->now() - current_state_monitor_->getMonitorStartTime()).seconds() > 1.0)
//     {
//       std::string missing_str = boost::algorithm::join(missing, ", ");
//       // std::string missing_str = std::accumulate(std::begin(missing), std::end(missing), std::string(), []
//       // (std::string &ss, std::string &s){return ss.empty() ? s : ss + "," + s});  // non-boost variation
//       RCLCPP_WARN_ONCE(
//           node_->get_logger(), "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
//     }
//
//     {
//       boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//       last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
//       RCLCPP_DEBUG(node_->get_logger(), "robot state update %f ", fmod(last_robot_motion_time_.seconds(), 10.));
//
//       tesseract_->getEnvironment()->setState(current_state_monitor_->getCurrentState()->joints);
//     }
//     triggerEnvironmentUpdateEvent(UPDATE_STATE);
//   }
//   else
//     RCLCPP_ERROR_ONCE(node_->get_logger(), "State monitor is not active. Unable to set the planning scene state");
// }
//
// void EnvironmentMonitor::addUpdateCallback(const boost::function<void(EnvironmentUpdateType)>& fn)
// {
//   boost::recursive_mutex::scoped_lock lock(update_lock_);
//   if (fn)
//     update_callbacks_.push_back(fn);
// }
//
// void EnvironmentMonitor::clearUpdateCallbacks()
// {
//   boost::recursive_mutex::scoped_lock lock(update_lock_);
//   update_callbacks_.clear();
// }
//
// void EnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
// {
//   publish_environment_frequency_ = hz;
//   RCLCPP_DEBUG(node_->get_logger(),
//                "Maximum frquency for publishing an environment is now %lf Hz",
//                publish_environment_frequency_);
// }
//
// void EnvironmentMonitor::modifyEnvironmentCallback(
//     const std::shared_ptr<tesseract_msgs::srv::ModifyEnvironment::Request> req,
//     std::shared_ptr<tesseract_msgs::srv::ModifyEnvironment::Response> res)
// {
//   res->success = applyEnvironmentCommandsMessage(req->id, static_cast<int>(req->revision), req->commands);
//   res->revision = static_cast<unsigned long>(tesseract_->getEnvironmentConst()->getRevision());
// }
//
// void EnvironmentMonitor::getEnvironmentChangesCallback(
//     const std::shared_ptr<tesseract_msgs::srv::GetEnvironmentChanges::Request> req,
//     std::shared_ptr<tesseract_msgs::srv::GetEnvironmentChanges::Response> res)
// {
//   if (static_cast<int>(req->revision) > tesseract_->getEnvironment()->getRevision())
//   {
//     res->success = false;
//     return;
//   }
//
//   res->id = tesseract_->getEnvironment()->getName();
//   res->revision = static_cast<unsigned long>(tesseract_->getEnvironment()->getRevision());
//   if (!tesseract_rosutils::toMsg(res->commands, tesseract_->getEnvironment()->getCommandHistory(), req->revision))
//   {
//     res->success = false;
//     return;
//   }
//
//   res->success = true;
// }
//
// void EnvironmentMonitor::getEnvironmentInformationCallback(
//     const std::shared_ptr<tesseract_msgs::srv::GetEnvironmentInformation::Request> req,
//     std::shared_ptr<tesseract_msgs::srv::GetEnvironmentInformation::Response> res)
// {
//   res->id = tesseract_->getEnvironment()->getName();
//   res->revision = static_cast<unsigned long>(tesseract_->getEnvironment()->getRevision());
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::COMMAND_HISTORY)
//   {
//     if (!tesseract_rosutils::toMsg(res->command_history, tesseract_->getEnvironment()->getCommandHistory(), 0))
//     {
//       res->success = false;
//       return;
//     }
//   }
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_LIST)
//   {
//     for (const auto& link : tesseract_->getEnvironmentConst()->getSceneGraph()->getLinks())
//     {
//       tesseract_msgs::msg::Link msg;
//       if (!tesseract_rosutils::toMsg(msg, *link))
//       {
//         res->success = false;
//         return;
//       }
//       res->links.push_back(msg);
//     }
//   }
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_LIST)
//   {
//     for (const auto& joint : tesseract_->getEnvironmentConst()->getSceneGraph()->getJoints())
//     {
//       tesseract_msgs::msg::Joint msg;
//       if (!tesseract_rosutils::toMsg(msg, *joint))
//       {
//         res->success = false;
//         return;
//       }
//       res->joints.push_back(msg);
//     }
//   }
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_NAMES)
//   {
//     for (const auto& link : tesseract_->getEnvironmentConst()->getLinkNames())
//     {
//       res->link_names.push_back(link);
//     }
//   }
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::JOINT_NAMES)
//   {
//     for (const auto& joint : tesseract_->getEnvironmentConst()->getJointNames())
//     {
//       res->joint_names.push_back(joint);
//     }
//   }
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ACTIVE_LINK_NAMES)
//   {
//     for (const auto& link : tesseract_->getEnvironmentConst()->getActiveLinkNames())
//     {
//       res->active_link_names.push_back(link);
//     }
//   }
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::ACTIVE_JOINT_NAMES)
//   {
//     for (const auto& joint : tesseract_->getEnvironmentConst()->getActiveJointNames())
//     {
//       res->active_joint_names.push_back(joint);
//     }
//   }
//
//   if (req->flags & tesseract_msgs::srv::GetEnvironmentInformation::Request::LINK_TRANSFORMS)
//   {
//     for (const auto& link_pair : tesseract_->getEnvironmentConst()->getCurrentState()->link_transforms)
//     {
//       res->link_transforms.names.push_back(link_pair.first);
//       geometry_msgs::msg::Pose pose = tf2::toMsg(link_pair.second);
//       res->link_transforms.transforms.push_back(pose);
//     }
//   }
//
//   res->success = true;
// }
//
// }  // namespace tesseract_monitoring
