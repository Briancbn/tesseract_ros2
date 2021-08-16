cmake_minimum_required(VERSION 3.5.0)
project(tesseract_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -std=c++11
    -Wall
    -Wextra
    -Wsuggest-override
    -Wconversion
    -Wsign-conversion)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(msg_files
    "msg/AllowedCollisionEntry.msg"
    "msg/ChainGroup.msg"
    "msg/CollisionGeometry.msg"
    "msg/CollisionMarginData.msg"
    "msg/CollisionMarginOverrideType.msg"
    "msg/ContactMarginPair.msg"
    "msg/ContactResult.msg"
    "msg/ContactResultVector.msg"
    "msg/Environment.msg"
    "msg/EnvironmentCommand.msg"
    "msg/EnvironmentState.msg"
    "msg/Geometry.msg"
    "msg/GroupsJointState.msg"
    "msg/GroupsJointStates.msg"
    "msg/GroupsOPWKinematics.msg"
    "msg/GroupsREPKinematics.msg"
    "msg/GroupsROPKinematics.msg"
    "msg/GroupsTCP.msg"
    "msg/GroupsTCPs.msg"
    "msg/Inertial.msg"
    "msg/Joint.msg"
    "msg/JointCalibration.msg"
    "msg/JointDynamics.msg"
    "msg/JointGroup.msg"
    "msg/JointLimits.msg"
    "msg/JointMimic.msg"
    "msg/JointSafety.msg"
    "msg/JointState.msg"
    "msg/KinematicsInformation.msg"
    "msg/Link.msg"
    "msg/LinkGroup.msg"
    "msg/Material.msg"
    "msg/Mesh.msg"
    "msg/ObjectColor.msg"
    "msg/OctreeSubType.msg"
    "msg/PlannerProfileRemapping.msg"
    "msg/PlanningRequest.msg"
    "msg/PlanningRequestArchive.msg"
    "msg/PlanningResponse.msg"
    "msg/PlanningResponseArchive.msg"
    "msg/ProfileMap.msg"
    "msg/SceneGraph.msg"
    "msg/StringDoublePair.msg"
    "msg/StringLimitsPair.msg"
    "msg/StringPair.msg"
    "msg/TaskInfo.msg"
    "msg/Trajectory.msg"
    "msg/TransformMap.msg"
    "msg/VisualGeometry.msg"
)

set(srv_files
    "srv/ComputeContactResultVector.srv"
    "srv/GetEnvironmentChanges.srv"
    "srv/GetEnvironmentInformation.srv"
    "srv/ModifyEnvironment.srv"
    "srv/SaveSceneGraph.srv"
)

set(action_files
    "action/GetMotionPlan.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs shape_msgs octomap_msgs trajectory_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
