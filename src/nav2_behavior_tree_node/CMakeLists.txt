cmake_minimum_required(VERSION 3.5)
project(behavior_tree_ex_control)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  nav2_util
)

add_library(${library_name} SHARED
  plugins/control/recovery_node.cpp
  plugins/control/pipeline_sequence.cpp
  plugins/control/round_robin_node.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_library(nav2_recovery_node_bt_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs nav2_recovery_node_bt_node)

ament_target_dependencies(
  nav2_recovery_node_bt_node
  ${dependencies}
)

add_library(nav2_pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs nav2_pipeline_sequence_bt_node)


ament_target_dependencies(
  nav2_pipeline_sequence_bt_node
  ${dependencies}
)

add_library(nav2_round_robin_node_bt_node SHARED plugins/control/round_robin_node.cpp)

ament_target_dependencies(
  nav2_round_robin_node_bt_node
  ${dependencies}
)

add_library(nav2_pipeline_sequence SHARED plugins/control/pipeline_sequence.cpp)

ament_target_dependencies(
  nav2_pipeline_sequence
  ${dependencies}
)

list(APPEND plugin_libs nav2_round_robin_node_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS nav2_pipeline_sequence
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES nav2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
