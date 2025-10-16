cmake_minimum_required(VERSION 3.8)
project(visual_icons)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(icon_publisher src/icon_publisher.cpp)
ament_target_dependencies(icon_publisher rclcpp visualization_msgs)

# Install meshes/textures so "package://" URIs work
install(DIRECTORY meshes/
        DESTINATION share/${PROJECT_NAME}/meshes)

install(TARGETS icon_publisher
        DESTINATION lib/${PROJECT_NAME})

ament_package()
