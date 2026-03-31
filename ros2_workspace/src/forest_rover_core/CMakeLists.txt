cmake_minimum_required(VERSION 3.8)
project(forest_rover_core)

# Metapackage definition
find_package(ament_cmake REQUIRED)

# Find all sub-packages
find_package(forest_rover_hardware REQUIRED)
find_package(forest_rover_perception REQUIRED)
find_package(forest_rover_autonomy REQUIRED)
find_package(forest_rover_description REQUIRED)
find_package(forest_rover_msgs REQUIRED)
find_package(forest_rover_utils REQUIRED)

# Uncomment to enable linting
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()

ament_package()
