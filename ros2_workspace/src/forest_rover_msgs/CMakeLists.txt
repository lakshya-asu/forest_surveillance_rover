cmake_minimum_required(VERSION 3.8)
project(forest_rover_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
	"msg/MotorCommand.msg"
	"msg/MotorFeedback.msg"
	"msg/EnvironmentalData.msg"
	"msg/Detection.msg"
	"msg/DetectionArray.msg"
	"msg/RoverEvent.msg"
)

set(srv_files
	"srv/EmergencyStop.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
	${srv_files}
	DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
