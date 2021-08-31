/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
// #include <ros/console.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <tesseract_msgs/msg/tesseract_state.hpp>
#include <tesseract_msgs/msg/environment_command.hpp>
#include <tesseract_msgs/msg/geometry.hpp>
#include <tesseract_msgs/msg/visual_geometry.hpp>
#include <tesseract_msgs/msg/collision_geometry.hpp>

#include <tesseract_msgs/srv/modify_environment.hpp>
#include <tesseract_msgs/srv/get_environment_changes.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

static const std::string ROBOT_DESCRIPTION = "robot_description";

rclcpp::Client<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_env_rviz_client;
rclcpp::Client<tesseract_msgs::srv::GetEnvironmentChanges>::SharedPtr get_env_changes_rviz_client;

rclcpp::Client<tesseract_msgs::srv::ModifyEnvironment>::SharedPtr modify_env_master_client;
rclcpp::Client<tesseract_msgs::srv::GetEnvironmentChanges>::SharedPtr get_env_changes_master_client;

rclcpp::Node::SharedPtr node;

void addSphere(const std::string& name, const std::string id, int& revision)
{
  // tesseract_msgs::srv::ModifyEnvironment update_env;
  auto update_env = std::make_shared<tesseract_msgs::srv::ModifyEnvironment::Request>();
  // Create add command
  tesseract_msgs::msg::EnvironmentCommand add_sphere_command;
  add_sphere_command.command = tesseract_msgs::msg::EnvironmentCommand::ADD_LINK;
  // Create the link
  add_sphere_command.add_link.name = name;

  tesseract_msgs::msg::VisualGeometry visual;
  visual.origin.position.x = 0.5;
  visual.origin.position.y = 0;
  visual.origin.position.z = 0.55;
  visual.material.empty = true;

  visual.geometry.type = tesseract_msgs::msg::Geometry::SPHERE;
  visual.geometry.sphere_radius = 0.15;

  add_sphere_command.add_link.visual.push_back(visual);

  tesseract_msgs::msg::CollisionGeometry collision;
  collision.origin.position.x = 0.5;
  collision.origin.position.y = 0;
  collision.origin.position.z = 0.55;
  collision.material.empty = true;

  collision.geometry.type = tesseract_msgs::msg::Geometry::SPHERE;
  collision.geometry.sphere_radius = 0.15;

  add_sphere_command.add_link.collision.push_back(collision);

  // Create the Joint
  add_sphere_command.add_joint.type = tesseract_msgs::msg::Joint::FIXED;
  add_sphere_command.add_joint.name = name + "_joint";
  add_sphere_command.add_joint.parent_link_name = "base_link";
  add_sphere_command.add_joint.child_link_name = name;

  update_env->id = id;
  update_env->revision = revision;
  update_env->commands.push_back(add_sphere_command);

  auto modify_env_rviz_future = modify_env_rviz_client->async_send_request(update_env);
  if (rclcpp::spin_until_future_complete(node, modify_env_rviz_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to add sphere to environment");
    return;
  }
  auto result = modify_env_rviz_future.get();
  RCLCPP_INFO(node->get_logger(), "Sphere added to Environment!");
  revision = result->revision;

  // if (modify_env_rviz_client->call(update_env))
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Sphere added to Environment!");
  //   revision = update_env.response.revision;
  // }
  // else
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Failed to add sphere to environment");
  // }
}

void removeSphere(const std::string& name, const std::string id, int& revision)
{
  // tesseract_msgs::srv::ModifyEnvironment update_env;
  auto update_env = std::make_shared<tesseract_msgs::srv::ModifyEnvironment::Request>();

  // Create remove command
  tesseract_msgs::msg::EnvironmentCommand command;
  command.command = tesseract_msgs::msg::EnvironmentCommand::REMOVE_LINK;
  command.remove_link = name;

  update_env->id = id;
  update_env->revision = revision;
  update_env->commands.push_back(command);

  auto modify_env_rviz_future = modify_env_rviz_client->async_send_request(update_env);
  if (rclcpp::spin_until_future_complete(node, modify_env_rviz_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to remove sphere to environment");
    return;
  }
  auto result = modify_env_rviz_future.get();
  RCLCPP_INFO(node->get_logger(), "Removed sphere from environment!");
  revision = result->revision;

  // if (modify_env_rviz_client.call(update_env))
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Removed sphere from environment!");
  //   revision = update_env.response.revision;
  // }
  // else
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Failed to remove sphere from environment");
  // }
}

bool updateRViz()
{
  // get_env_changes_rviz.waitForExistence();
  while (!get_env_changes_rviz_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "get_env_changes_rviz_client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for get_env_changes_rviz_client service to appear...");
  }

  // Get the current state of the environment
  // tesseract_msgs::srv::GetEnvironmentChanges env_changes;
  auto env_changes = std::make_shared<tesseract_msgs::srv::GetEnvironmentChanges::Request>();
  env_changes->revision = 0;

  auto get_env_changes_rviz_future = get_env_changes_rviz_client->async_send_request(env_changes);
  if (rclcpp::spin_until_future_complete(node, get_env_changes_rviz_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed retrieve current environment changes!");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Retrieve current environment changes!");
  auto result = get_env_changes_rviz_future.get();
  // if (get_env_changes_rviz_client.call(env_changes))
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Retrieve current environment changes!");
  // }
  // else
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Failed retrieve current environment changes!");
  //   return false;
  // }

  std::string id = result->id;
  int revision = result->revision;

  // modify_env_rviz_client.waitForExistence();
  while (!modify_env_rviz_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "modify_env_rviz_client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for modify_env_rviz_client service to appear...");
  }

  addSphere("sphere_attached", id, revision);
  rclcpp::sleep_for(std::chrono::seconds(10));
  removeSphere("sphere_attached", id, revision);

  return true;
}

bool updateMaster()
{
  // get_env_changes_master_client.waitForExistence();
  while (!get_env_changes_master_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "get_env_changes_master_client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for get_env_changes_master_client service to appear...");
  }
  // Get the current state of the environment
  // tesseract_msgs::srv::GetEnvironmentChanges env_changes;
  auto env_changes = std::make_shared<tesseract_msgs::srv::GetEnvironmentChanges::Request>();
  env_changes->revision = 0;

  auto get_env_changes_rviz_future = get_env_changes_rviz_client->async_send_request(env_changes);
  if (rclcpp::spin_until_future_complete(node, get_env_changes_rviz_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed retrieve current environment changes!");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Retrieve current environment changes!");
  auto result = get_env_changes_rviz_future.get();
  // if (get_env_changes_master_client.call(env_changes))
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Retrieve current environment changes!");
  // }
  // else
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Failed retrieve current environment changes!");
  //   return false;
  // }

  std::string id = result->id;
  int revision = result->revision;

  // modify_env_master_client.waitForExistence();
  while (!modify_env_master_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "modify_env_master_client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for modify_env_master_client service to appear...");
  }

  addSphere("sphere_attached", id, revision);
  rclcpp::sleep_for(std::chrono::seconds(10));
  removeSphere("sphere_attached", id, revision);

  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ros::init(argc, argv, "demo", ros::init_options::AnonymousName);
  node = rclcpp::Node::make_shared("tesseract_monitor_demo");

  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  using rclcpp::executors::MultiThreadedExecutor;
  MultiThreadedExecutor executor;
  executor.add_node(node);

  // These are used to keep visualization updated
  // modify_env_rviz = node->serviceClient<tesseract_msgs::srv::ModifyEnvironment>("modify_tesseract_rviz", 10);
  // get_env_changes_rviz = node->serviceClient<tesseract_msgs::srv::GetEnvironmentChanges>("get_tesseract_changes_rviz", 10);
  modify_env_rviz_client = node->create_client<tesseract_msgs::srv::ModifyEnvironment>("modify_tesseract_rviz");
  get_env_changes_rviz_client = node->create_client<tesseract_msgs::srv::GetEnvironmentChanges>("get_tesseract_changes_rviz");

  // These are used to keep master version of the environment updated
  // modify_env_master = node->serviceClient<tesseract_msgs::srv::ModifyEnvironment>("modify_tesseract", 10);
  // get_env_changes_master = node->serviceClient<tesseract_msgs::srv::GetEnvironmentChanges>("get_tesseract_changes", 10);
  modify_env_master_client = node->create_client<tesseract_msgs::srv::ModifyEnvironment>("modify_tesseract");
  get_env_changes_master_client = node->create_client<tesseract_msgs::srv::GetEnvironmentChanges>("get_tesseract_changes");

  if (!updateRViz())
    return -1;

  if (!updateMaster())
    return -1;

  rclcpp::shutdown();
  return 0;
}
