/**
 * @file scene_graph_example.h
 * @brief This example initializes 2 robots from a URDF. It then reattaches one of the robots to the end effector of the
 * other robot
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
#ifndef TESSERACT_ROS_EXAMPLES_SCENE_GRAPH_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_SCENE_GRAPH_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

namespace tesseract_ros_examples
{
/**
 * @brief Basic example leveraging trajopt and tesseract for cartesian planning
 */
class SceneGraphExample : public Example
{
public:
  SceneGraphExample(rclcpp::Node::SharedPtr node, bool plotting, bool rviz) : Example(plotting, rviz), node_(node) {}
  ~SceneGraphExample() override = default;
  SceneGraphExample(const SceneGraphExample&) = default;
  SceneGraphExample& operator=(const SceneGraphExample&) = default;
  SceneGraphExample(SceneGraphExample&&) = default;
  SceneGraphExample& operator=(SceneGraphExample&&) = default;

  bool run() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
