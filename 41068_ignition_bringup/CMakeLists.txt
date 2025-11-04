cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)  # ok to keep even if not used directly

# ---- Install data folders (used at runtime) ----
# Add 'maps' if you will generate PGM/YAML maps. Keep a .gitkeep inside empty dirs.

find_package(ros_ign_interfaces REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)


install(
  DIRECTORY config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# ---- Install executable Python scripts (so `ros2 run` works) ----
# Make sure scripts have a shebang (#!/usr/bin/env python3).
# Install data folders (already present, keep it)

# NEW: install Python executables so `ros2 run` can find them
install(
  PROGRAMS
    scripts/world_gen.py
    scripts/model_list.py
    scripts/thermal_classifier.py
    scripts/thermal_detection.py
  DESTINATION lib/${PROJECT_NAME}

)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
