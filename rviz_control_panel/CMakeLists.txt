cmake_minimum_required(VERSION 3.8)
project(rviz_control_panel)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Avoid Qt macro collisions
add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/panel.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  pluginlib
  std_msgs
  nav2_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

# Export as an RViz plugin
pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(FILES rviz_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
