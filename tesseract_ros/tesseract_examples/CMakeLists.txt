cmake_minimum_required(VERSION 3.5.0)
project(tesseract_ros_examples VERSION 0.1.0 LANGUAGES CXX)

# ROS packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tesseract_monitoring REQUIRED)
find_package(tesseract_rosutils REQUIRED)
#find_package(tesseract_planning_server REQUIRED)
# find_package(octomap_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
find_package(tesseract_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)

list(FIND CMAKE_CXX_COMPILE_FEATURES cxx_std_17 CXX_FEATURE_FOUND)

include_directories(
  PUBLIC
  include
)

#tesseract_variables()

# add_library(${PROJECT_NAME}_basic_cartesian_example SHARED src/basic_cartesian_example.cpp)
# target_link_libraries(${PROJECT_NAME}_basic_cartesian_example PUBLIC
#     tesseract::tesseract_environment_core
#     tesseract::tesseract_environment_ofkt
#     tesseract::tesseract_process_managers
#     ${PCL_LIBRARIES}
#     ${rclcpp_LIBRARIES}
#     ${ament_index_cpp_LIBRARIES}
#     )
# # target_link_libraries(${PROJECT_NAME} nt_ofkt PUBLIC
# #     tesseract::tesseract_process_managers
# #     ${rclcpp_LIBRARIES}
# #     ${ament_index_cpp_LIBRARIES}
# #     )
# target_compile_options(${PROJECT_NAME}_basic_cartesian_example PUBLIC
#     ${TESSERACT_COMPILE_OPTIONS}
#     )
# # target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example ARGUMENTS
# #     ${TESSERACT_CLANG_TIDY_ARGS} ENABLE
# #     ${TESSERACT_ENABLE_CLANG_TIDY}
# #     )

# if(CXX_FEATURE_FOUND EQUAL "-1")
#     target_compile_options(${PROJECT_NAME} PUBLIC -std=c++17)
# else()
#     target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# endif()

# foreach(DEF ${PCL_DEFINITIONS})
#   string(STRIP ${DEF} DEF)
#   if (NOT "${DEF}" STREQUAL "")
#     string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
#     if ("${DEF_PREFIX}" STREQUAL "-m")
#       string(REPLACE " " ";" DEF ${DEF})
#       foreach(OPTION_DEF ${DEF})
#         target_compile_options(${PROJECT_NAME}_basic_cartesian_example PUBLIC ${OPTION_DEF})
#       endforeach()
#     else()
#       target_compile_definitions(${PROJECT_NAME}_basic_cartesian_example PUBLIC ${DEF})
#     endif()
#   endif()
# endforeach()
#
# target_include_directories(${PROJECT_NAME}_basic_cartesian_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_basic_cartesian_example SYSTEM PUBLIC
#     ${rclcpp_LIBRARIES}
#     ${ament_index_cpp_LIBRARIES}
#     ${PCL_INCLUDE_DIRS}
#   )
#
# add_executable(${PROJECT_NAME}_basic_cartesian_example_node src/basic_cartesian_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_node
#     ${PROJECT_NAME}_basic_cartesian_example
#     ${rclcpp_LIBRARIES}
#     ${ament_index_cpp_LIBRARIES}
#     )
# target_compile_options(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE
#     ${TESSERACT_COMPILE_OPTIONS}
#     )
# # target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_node ARGUMENTS
# #     ${TESSERACT_CLANG_TIDY_ARGS} ENABLE
# #     ${TESSERACT_ENABLE_CLANG_TIDY}
# #     )
# # target_cxx_version(${PROJECT_NAME}_basic_cartesian_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_basic_cartesian_example_node SYSTEM PRIVATE
#     ${rclcpp_LIBRARIES}
#     ${ament_index_cpp_LIBRARIES}
#     )

# add_library(${PROJECT_NAME}_freespace_ompl_example src/freespace_ompl_example.cpp)
# target_link_libraries(${PROJECT_NAME}_freespace_ompl_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_freespace_ompl_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_freespace_ompl_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_freespace_ompl_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_freespace_ompl_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_freespace_ompl_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_freespace_ompl_example_node src/freespace_ompl_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_freespace_ompl_example_node ${PROJECT_NAME}_freespace_ompl_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_freespace_ompl_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_freespace_ompl_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_freespace_ompl_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_freespace_ompl_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_freespace_ompl_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})
#
# add_library(${PROJECT_NAME}_freespace_hybrid_example src/freespace_hybrid_example.cpp)
# target_link_libraries(${PROJECT_NAME}_freespace_hybrid_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_freespace_hybrid_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_freespace_hybrid_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_freespace_hybrid_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_freespace_hybrid_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_freespace_hybrid_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_freespace_hybrid_example_node src/freespace_hybrid_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_freespace_hybrid_example_node ${PROJECT_NAME}_freespace_hybrid_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_freespace_hybrid_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_freespace_hybrid_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_freespace_hybrid_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_freespace_hybrid_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_freespace_hybrid_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})
#
# add_library(${PROJECT_NAME}_car_seat_example src/car_seat_example.cpp)
# target_link_libraries(${PROJECT_NAME}_car_seat_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_car_seat_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_car_seat_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_car_seat_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_car_seat_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_car_seat_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_car_seat_example_node src/car_seat_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_car_seat_example_node ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_car_seat_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_car_seat_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_car_seat_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_car_seat_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_car_seat_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})
#
# add_library(${PROJECT_NAME}_glass_upright_example src/glass_upright_example.cpp)
# target_link_libraries(${PROJECT_NAME}_glass_upright_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_glass_upright_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_glass_upright_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_glass_upright_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_glass_upright_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_glass_upright_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_glass_upright_example_node src/glass_upright_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_glass_upright_example_node ${PROJECT_NAME}_glass_upright_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_glass_upright_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_glass_upright_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_glass_upright_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_glass_upright_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_glass_upright_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})




## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  add_library(${PROJECT_NAME}_glass_upright_ompl_example src/glass_upright_ompl_example.cpp)
#  target_link_libraries(${PROJECT_NAME}_glass_upright_ompl_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt trajopt::trajopt tesseract::tesseract_motion_planners_ompl tesseract::tesseract_motion_planners_trajopt jsoncpp_lib ${catkin_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
#  target_clang_tidy(${PROJECT_NAME}_glass_upright_ompl_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#  target_cxx_version(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC VERSION 17)
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example PUBLIC
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example SYSTEM PUBLIC
#      ${catkin_INCLUDE_DIRS})

#  add_executable(${PROJECT_NAME}_glass_upright_ompl_example_node src/glass_upright_ompl_example_node.cpp)
#  target_link_libraries(${PROJECT_NAME}_glass_upright_ompl_example_node ${PROJECT_NAME}_glass_upright_ompl_example ${catkin_LIBRARIES})
#  target_compile_options(${PROJECT_NAME}_glass_upright_ompl_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#  target_clang_tidy(${PROJECT_NAME}_glass_upright_ompl_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#  target_cxx_version(${PROJECT_NAME}_glass_upright_ompl_example_node PUBLIC VERSION 17)
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example_node PRIVATE
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
#  target_include_directories(${PROJECT_NAME}_glass_upright_ompl_example_node SYSTEM PRIVATE
#      ${catkin_INCLUDE_DIRS})
#endif()




# add_library(${PROJECT_NAME}_online_planning_example src/online_planning_example.cpp)
# target_link_libraries(${PROJECT_NAME}_online_planning_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt trajopt::trajopt_ifopt trajopt::trajopt_sqp ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_online_planning_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_online_planning_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_online_planning_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_online_planning_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_online_planning_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_online_planning_example_node src/online_planning_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_online_planning_example_node ${PROJECT_NAME}_online_planning_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_online_planning_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_online_planning_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_online_planning_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_online_planning_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_online_planning_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})
#
# add_library(${PROJECT_NAME}_pick_and_place_example src/pick_and_place_example.cpp)
# target_link_libraries(${PROJECT_NAME}_pick_and_place_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_pick_and_place_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_pick_and_place_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_pick_and_place_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_pick_and_place_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_pick_and_place_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_pick_and_place_example_node src/pick_and_place_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_pick_and_place_example_node ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_pick_and_place_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_pick_and_place_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_pick_and_place_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_pick_and_place_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_pick_and_place_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})
#
# add_library(${PROJECT_NAME}_puzzle_piece_example src/puzzle_piece_example.cpp)
# target_link_libraries(${PROJECT_NAME}_puzzle_piece_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_puzzle_piece_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_puzzle_piece_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_puzzle_piece_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_puzzle_piece_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_puzzle_piece_example_node src/puzzle_piece_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_node ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_puzzle_piece_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})
#
# add_library(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example.cpp)
# target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_motion_planners_trajopt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example PUBLIC
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example SYSTEM PUBLIC
#     ${catkin_INCLUDE_DIRS})
#
# add_executable(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node src/puzzle_piece_auxillary_axes_example_node.cpp)
# target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
# target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
# target_cxx_version(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PUBLIC VERSION 17)
# target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node PRIVATE
#     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#     "$<INSTALL_INTERFACE:include>")
# target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_node SYSTEM PRIVATE
#     ${catkin_INCLUDE_DIRS})
#
add_library(${PROJECT_NAME}_scene_graph_example src/scene_graph_example.cpp)
target_link_libraries(
    ${PROJECT_NAME}_scene_graph_example
    tesseract::tesseract_environment_core
    tesseract::tesseract_environment_ofkt
    tesseract::tesseract_motion_planners_trajopt
    jsoncpp_lib
    ${rclcpp_LIBRARIES}
    ${tesseract_msgs_LIBRARIES}
    ${tesseract_monitoring_LIBRARIES}
    ${tesseract_rosutils_LIBRARIES}
    ${tesseract_process_managers_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME}_scene_graph_example PUBLIC ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_scene_graph_example ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_scene_graph_example PRIVATE -std=c++17)
else()
    target_compile_features(${PROJECT_NAME}_scene_graph_example PRIVATE cxx_std_17)
endif()
target_include_directories(
    ${PROJECT_NAME}_scene_graph_example PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_scene_graph_example SYSTEM PUBLIC
                           ${rclcpp_INCLUDE_DIRS}
                           ${tesseract_msgs_INCLUDE_DIRS}
                           ${tesseract_monitoring_INCLUDE_DIRS}
                           ${tesseract_rosutils_INCLUDE_DIRS}
                           ${tesseract_process_managers_INCLUDE_DIRS}
                          )
list(APPEND PACKAGE_FILES ${PROJECT_NAME}_scene_graph_example)


add_executable(${PROJECT_NAME}_scene_graph_example_node src/scene_graph_example_node.cpp)
target_link_libraries(${PROJECT_NAME}_scene_graph_example_node
                      ${PROJECT_NAME}_scene_graph_example
                      ${rclcpp_LIBRARIES}
                      ${tesseract_monitoring_LIBRARIES}
                      )
target_compile_options(${PROJECT_NAME}_scene_graph_example_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
# target_clang_tidy(${PROJECT_NAME}_scene_graph_example_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_scene_graph_example_node PRIVATE -std=c++17)
else()
    target_compile_features(${PROJECT_NAME}_scene_graph_example_node PRIVATE cxx_std_17)
endif()
target_include_directories(
    ${PROJECT_NAME}_scene_graph_example_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
target_include_directories(
    ${PROJECT_NAME}_scene_graph_example_node SYSTEM PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${tesseract_monitoring_INCLUDE_DIRS}
    )
list(APPEND PACKAGE_FILES ${PROJECT_NAME}_scene_graph_example_node)

ament_export_include_directories(include)

ament_export_dependencies(
    geometry_msgs
    tesseract_msgs
    rclcpp
    ament_index_cpp
    tesseract_monitoring
    tesseract_rosutils
    #tesseract_planning_server
    pcl_conversions
    # octomap_ros
    tesseract_environment
    tesseract_command_language
    tesseract_motion_planners
    tesseract_process_managers
    tesseract_common
    trajopt_sqp
    trajopt_ifopt
    ros_industrial_cmake_boilerplate
    PCL
    )

ament_export_libraries(${PROJECT_NAME})
ament_package()

install(
    TARGETS ${PACKAGE_FILES}
    EXPORT ${PROJECT_NAME}-targets
    DESTINATION lib
    )
install(
    EXPORT ${PROJECT_NAME}-targets
    NAMESPACE tesseract::
    DESTINATION lib/cmake/${PROJECT_NAME}
    )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)
  find_package(tesseract_support REQUIRED)

#  ament_add_gtest(${PROJECT_NAME}_unit test/tesseract_rosutils_unit.cpp)
#  target_link_libraries(${PROJECT_NAME}_unit GTest::GTest GTest::Main tesseract::tesseract_support ${PROJECT_NAME})
#	if(CXX_FEATURE_FOUND EQUAL "-1")
#            target_compile_options(${PROJECT_NAME}_unit PRIVATE -std=c++17)
#	else()
#            target_compile_features(${PROJECT_NAME}_unit PRIVATE cxx_std_17)
#	endif()
#  target_include_directories(${PROJECT_NAME}_unit PRIVATE
#      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#      "$<INSTALL_INTERFACE:include>")
endif()

# if(CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     find_package(GTest REQUIRED)
#
#     add_rostest_gtest(${PROJECT_NAME}_basic_cartesian_example_unit test/basic_cartesian_example_unit.test test/basic_cartesian_example_unit.cpp)
#     target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
#     target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#     target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#     target_cxx_version(${PROJECT_NAME}_basic_cartesian_example_unit PUBLIC VERSION 17)
#     target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>")
#     target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit SYSTEM PRIVATE
#         ${catkin_INCLUDE_DIRS})
#
#     add_rostest_gtest(${PROJECT_NAME}_car_seat_example_unit test/car_seat_example_unit.test test/car_seat_example_unit.cpp)
#     target_link_libraries(${PROJECT_NAME}_car_seat_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_car_seat_example ${catkin_LIBRARIES})
#     target_compile_options(${PROJECT_NAME}_car_seat_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#     target_clang_tidy(${PROJECT_NAME}_car_seat_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#     target_cxx_version(${PROJECT_NAME}_car_seat_example_unit PUBLIC VERSION 17)
#     target_include_directories(${PROJECT_NAME}_car_seat_example_unit PRIVATE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>")
#     target_include_directories(${PROJECT_NAME}_car_seat_example_unit SYSTEM PRIVATE
#         ${catkin_INCLUDE_DIRS})
#
#     add_rostest_gtest(${PROJECT_NAME}_glass_upright_example_unit test/glass_upright_example_unit.test test/glass_upright_example_unit.cpp)
#     target_link_libraries(${PROJECT_NAME}_glass_upright_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_glass_upright_example ${catkin_LIBRARIES})
#     target_compile_options(${PROJECT_NAME}_glass_upright_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#     target_clang_tidy(${PROJECT_NAME}_glass_upright_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#     target_cxx_version(${PROJECT_NAME}_glass_upright_example_unit PUBLIC VERSION 17)
#     target_include_directories(${PROJECT_NAME}_glass_upright_example_unit PRIVATE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>")
#     target_include_directories(${PROJECT_NAME}_glass_upright_example_unit SYSTEM PRIVATE
#         ${catkin_INCLUDE_DIRS})
#
#     add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_example_unit test/puzzle_piece_example_unit.test test/puzzle_piece_example_unit.cpp)
#     target_link_libraries(${PROJECT_NAME}_puzzle_piece_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_example ${catkin_LIBRARIES})
#     target_compile_options(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#     target_clang_tidy(${PROJECT_NAME}_puzzle_piece_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#     target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_unit PUBLIC VERSION 17)
#     target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit PRIVATE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>")
#     target_include_directories(${PROJECT_NAME}_puzzle_piece_example_unit SYSTEM PRIVATE
#         ${catkin_INCLUDE_DIRS})
#
#     add_rostest_gtest(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit test/puzzle_piece_auxillary_axes_example_unit.test test/puzzle_piece_auxillary_axes_example_unit.cpp)
#     target_link_libraries(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example ${catkin_LIBRARIES})
#     target_compile_options(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#     target_clang_tidy(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#     target_cxx_version(${PROJECT_NAME}_puzzle_piece_example_unit PUBLIC VERSION 17)
#     target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit PRIVATE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>")
#     target_include_directories(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example_unit SYSTEM PRIVATE
#         ${catkin_INCLUDE_DIRS})
#
#     add_rostest_gtest(${PROJECT_NAME}_pick_and_place_example_unit test/pick_and_place_example_unit.test test/pick_and_place_example_unit.cpp)
#     target_link_libraries(${PROJECT_NAME}_pick_and_place_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_pick_and_place_example ${catkin_LIBRARIES})
#     target_compile_options(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#     target_clang_tidy(${PROJECT_NAME}_pick_and_place_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#     target_cxx_version(${PROJECT_NAME}_pick_and_place_example_unit PUBLIC VERSION 17)
#     target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit PRIVATE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>")
#     target_include_directories(${PROJECT_NAME}_pick_and_place_example_unit SYSTEM PRIVATE
#         ${catkin_INCLUDE_DIRS})
#
#     add_rostest_gtest(${PROJECT_NAME}_scene_graph_example_unit test/scene_graph_example_unit.test test/scene_graph_example_unit.cpp)
#     target_link_libraries(${PROJECT_NAME}_scene_graph_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_scene_graph_example ${catkin_LIBRARIES})
#     target_compile_options(${PROJECT_NAME}_scene_graph_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
#     target_clang_tidy(${PROJECT_NAME}_scene_graph_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
#     target_cxx_version(${PROJECT_NAME}_scene_graph_example_unit PUBLIC VERSION 17)
#     target_include_directories(${PROJECT_NAME}_scene_graph_example_unit PRIVATE
#         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#         "$<INSTALL_INTERFACE:include>")
#     target_include_directories(${PROJECT_NAME}_scene_graph_example_unit SYSTEM PRIVATE
#         ${catkin_INCLUDE_DIRS})
# endif()
