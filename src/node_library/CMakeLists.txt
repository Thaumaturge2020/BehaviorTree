cmake_minimum_required(VERSION 3.8)
project(node_library)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


if(NOT ROOT)
  set(ROOT ${PROJECT_SOURCE_DIR}/)
endif()

add_definitions(-DROOT="${ROOT}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(behavior_tree_ex_control REQUIRED)
include_directories("/usr/include/eigen3")

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  behaviortree_cpp
  behaviortree_ros2
  std_msgs
  robot_msgs
  behavior_tree_ex_control
)

aux_source_directory(src/. plugin_cpp_list)

add_library(node_library SHARED ${plugin_cpp_list})


target_include_directories(node_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
list(APPEND plugin_libs node_library)


# ament_target_dependencies(node_library rclcpp rclcpp_action rclcpp_components robot_msgs std_msgs behaviortree_cpp behaviortree_ros2)

foreach(node_plugin ${plugin_libs})
  ament_target_dependencies(${node_plugin} ${dependencies})
  target_compile_definitions(${node_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


install(TARGETS
  node_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(${PROJECT_SOURCE_DIR}/include)

ament_export_libraries(${plugin_libs})

ament_export_dependencies(${dependencies})

#INSTALL( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
#    DESTINATION ${BTNODE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h*")

ament_package()
