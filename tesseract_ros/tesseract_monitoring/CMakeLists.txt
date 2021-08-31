cmake_minimum_required(VERSION 3.5.0)
project(tesseract_monitoring VERSION 0.1.0 LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tesseract_msgs REQUIRED)
find_package(tesseract_rosutils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_17 CXX_FEATURE_FOUND)

include_directories(
  PUBLIC
  include
)

# Tesseract ROS Nodes
add_library(
    ${PROJECT_NAME}_environment SHARED
    src/environment_monitor.cpp
    src/current_state_monitor.cpp)

target_link_libraries(
    ${PROJECT_NAME}_environment PUBLIC
    tesseract::tesseract_environment_core
    tesseract::tesseract_environment_ofkt
    tesseract::tesseract_rosutils
    pluginlib::pluginlib
    ${rclcpp_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    )

target_compile_options(${PROJECT_NAME}_environment PRIVATE ${TESSERACT_COMPILE_OPTIONS})
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_environment PRIVATE -std=c++17)
else()
    target_compile_features(${PROJECT_NAME}_environment PRIVATE cxx_std_17)
endif()

target_include_directories(
    ${PROJECT_NAME}_environment PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )

target_include_directories(
    ${PROJECT_NAME}_environment SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    )
# Link libraries
# ament_target_dependencies(
#   ${PROJECT_NAME}_environment
#   rclcpp
#   pluginlib
#   Eigen3
#
#   tesseract_environment
#   tesseract_rosutils
#   tesseract_common
#   tesseract_srdf
#   tesseract_urdf
#   tesseract_rosutils
#   tesseract_scene_graph
#   tesseract_collision
#   tesseract_geometry
#   tesseract_kinematics
#   tesseract_motion_planners
#   tesseract_command_language
#   tesseract_process_managers
#   visualization_msgs
#   tf2_ros
# )
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_environment)

add_library(${PROJECT_NAME}_contacts src/contact_monitor.cpp)
target_link_libraries(${PROJECT_NAME}_contacts PRIVATE
    tesseract::tesseract_environment_core
    ${PROJECT_NAME}_environment
    ${rclcpp_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME}_contacts PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_contacts PRIVATE -std=c++17)
else()
    target_compile_features(${PROJECT_NAME}_contacts PRIVATE cxx_std_17)
endif()
target_include_directories(${PROJECT_NAME}_contacts
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_contacts SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
)
# ament_target_dependencies(
#   ${PROJECT_NAME}_contacts
#     pluginlib
#   )
list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_contacts)

# add_library(${PROJECT_NAME}_environment SHARED
# #   src/contact_monitor_node.cpp
#     src/contact_monitor.cpp
#     src/current_state_monitor.cpp
# #   src/environment_monitor_interface.cpp
# #   src/environment_monitor_node.cpp
#     src/environment_monitor.cpp
#     )
# target_link_libraries(${PROJECT_NAME}_environment PUBLIC
#     ${rclcpp_LIBRARIES}
#     tesseract::tesseract_rosutils
#     tesseract::tesseract_collision_core
#     ${Boost_LIBRARIES}
#     ${tesseract_msgs_LIBRARIES}
#     ${tf2_ros_LIBRARIES}
#     ${pluginlib_LIBRARIES}
#     )
# target_compile_options(${PROJECT_NAME}_environment PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
# set_target_properties(${PROJECT_NAME}_environment PROPERTIES CXX_STANDARD 14)
# target_include_directories(${PROJECT_NAME}_environment PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_environment SYSTEM PUBLIC
#     ${rclcpp_INCLUDE_DIRS}
#     ${Boost_INCLUDE_DIRS}
#     ${tesseract_msgs_INCLUDE_DIRS}
#     ${tf2_ros_INCLUDE_DIRS}
#     ${pluginlib_INCLUDE_DIRS}
#     )
# list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_environment)

# Env monitor node
add_executable(${PROJECT_NAME}_environment_node src/environment_monitor_node.cpp)
target_link_libraries(
    ${PROJECT_NAME}_environment_node PRIVATE
    ${PROJECT_NAME}_environment
    ${rclcpp_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME}_environment_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_environment_node PRIVATE -std=c++17)
else()
    target_compile_features(${PROJECT_NAME}_environment_node PRIVATE cxx_std_17)
endif()
target_include_directories(${PROJECT_NAME}_environment_node
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_environment_node SYSTEM PUBLIC
                           ${rclcpp_INCLUDE_DIRS}
                          )

# Demo Node
add_executable(demo_scene demos/demo_scene.cpp)
target_link_libraries(
    demo_scene PRIVATE
    ${PROJECT_NAME}_environment
    ${rclcpp_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    )
target_compile_options(demo_scene PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(demo_scene PRIVATE -std=c++17)
else()
    target_compile_features(demo_scene PRIVATE cxx_std_17)
endif()
target_include_directories(demo_scene
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>)
target_include_directories(
    demo_scene SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    )

ament_export_dependencies(
    EIGEN3
    tesseract_msgs
    visualization_msgs
    rclcpp
    tesseract_rosutils
    pluginlib
    tf2_ros
    tf2_eigen
    )
ament_export_include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${tesseract_msgs_INCLUDE_DIRS}
    )
ament_export_libraries(${PROJECT_NAME}_environment)

ament_package()

message(INFO "Pkg list:" ${PACKAGE_LIBRARIES})

# Install libraries
install(
    TARGETS ${PACKAGE_LIBRARIES}
    EXPORT ${PROJECT_NAME}-targets
    DESTINATION lib)

install(
    EXPORT ${PROJECT_NAME}-targets
    NAMESPACE tesseract::
    DESTINATION lib/cmake/${PROJECT_NAME})

# Install executable(s)
install(TARGETS
   ${PROJECT_NAME}_environment_node
   demo_scene
   DESTINATION lib/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

# Mark executables and/or libraries for installation
#install(TARGETS
#    ${PROJECT_NAME}_environment
#    ${PROJECT_NAME}_contacts_node
#    ${PROJECT_NAME}_environment_node
##    demo_scene
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Create cmake config files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
