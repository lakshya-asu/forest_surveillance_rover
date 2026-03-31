cmake_minimum_required(VERSION 3.8)
project(base_station_receiver_node)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(forest_rover_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/base_station_receiver_main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
