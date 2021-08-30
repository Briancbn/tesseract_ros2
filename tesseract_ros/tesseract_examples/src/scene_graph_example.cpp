/**
 * @file scene_graph_example.cpp
 * @brief scene_graph_example implementation
 *
 * @author Matthew Powelson
 * @date Feb 17, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/scene_graph_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_monitoring/environment_monitor.h>


using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_rosutils;

namespace tesseract_ros_examples
{
bool SceneGraphExample::run()
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  // node_->get_parameter(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  // node_->get_parameter(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  urdf_xml_string = node_->get_parameter(ROBOT_DESCRIPTION_PARAM).as_string();
  srdf_xml_string = node_->get_parameter(ROBOT_SEMANTIC_PARAM).as_string();

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create monitor
  monitor_ = std::make_shared<tesseract_monitoring::EnvironmentMonitor>(env_, node_, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz_)
    monitor_->startPublishingEnvironment();

  // ros::spinOnce();
  {
    auto lock = monitor_->lockEnvironmentRead();
    monitor_->getEnvironment()->getSceneGraph()->saveDOT("scene_graph_example.dot");
  }

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(monitor_->getSceneGraph()->getRoot());

  if (plotting_)
  {
    plotter->waitForInput();
    RCLCPP_INFO(node_->get_logger(), "Reconfiguring using moveJoint");
  }

  // Attach the iiwa to the end of the ABB using moveJoint. Notice that the joint transform stays the same.
  // Only the parent changes.
  auto move_joint_cmd = std::make_shared<tesseract_environment::MoveJointCommand>("to_iiwa_mount", "tool0");
  monitor_->applyCommand(move_joint_cmd);
  //ros::spinOnce();

  // Save the scene graph to a file and publish the change
  {
    auto lock = monitor_->lockEnvironmentRead();
    monitor_->getEnvironment()->getSceneGraph()->saveDOT("scene_graph_example_moveJoint.dot");
  }

  if (plotting_)
  {
    plotter->waitForInput();
    RCLCPP_INFO(node_->get_logger(), "Reconfiguring using moveLink");
  }

  // Attach the iiwa to the end of the ABB using moveLink.
  // The link to be moved is inferred to be the given Joint child
  tesseract_scene_graph::Joint new_joint("to_iiwa_mount");
  new_joint.parent_link_name = "tool0";
  new_joint.child_link_name = "iiwa_mount";
  new_joint.type = tesseract_scene_graph::JointType::FIXED;
  new_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  new_joint.parent_to_joint_origin_transform.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(0, 1, 0)));
  new_joint.parent_to_joint_origin_transform.translate(Eigen::Vector3d(0.15, 0.0, 0.0));
  auto move_link_cmd = std::make_shared<tesseract_environment::MoveLinkCommand>(new_joint);
  monitor_->applyCommand(move_link_cmd);
  //ros::spinOnce();

  // Save the scene graph to a file and publish the change
  {
    auto lock = monitor_->lockEnvironmentRead();
    monitor_->getEnvironment()->getSceneGraph()->saveDOT("scene_graph_example_moveLink.dot");
  }

  if (plotting_)
  {
    plotter->waitForInput();
    RCLCPP_INFO(node_->get_logger(), "Open .dot files  in ~/.ros to see scene graph");
  }
  return true;
}
}  // namespace tesseract_ros_examples
