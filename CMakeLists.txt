cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "Include directories: ${CMAKE_CXX_INCLUDE_DIRS}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/GetDirection.srv"
	"action/GoToPose.action"
	DEPENDENCIES sensor_msgs geometry_msgs
)

add_executable(${PROJECT_NAME}_node src/patrol.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp geometry_msgs sensor_msgs nav_msgs)

add_executable(${PROJECT_NAME}_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(${PROJECT_NAME}_with_service_node rclcpp geometry_msgs sensor_msgs nav_msgs)

add_executable(laser_scan_subscriber_node src/laser_scan_subscriber.cpp)
ament_target_dependencies(laser_scan_subscriber_node rclcpp sensor_msgs)

add_executable(find_direction_buffers_node src/find_direction_buffers.cpp)
ament_target_dependencies(find_direction_buffers_node rclcpp sensor_msgs)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp sensor_msgs)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp sensor_msgs)

add_executable(go_to_pose_action_node src/go_to_pose_action.cpp)
ament_target_dependencies(go_to_pose_action_node rclcpp geometry_msgs nav_msgs)

# To link to interfaces defined and generated in the same package:

# Get the typesupport target name
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against the generated interface library
# NOTE: CMake, no keywords (PRIVATE, PUBLIC) or all keywords (can't mix)
target_link_libraries(direction_service_node ${typesupport_target})
target_link_libraries(test_service_node ${typesupport_target})
target_link_libraries(${PROJECT_NAME}_with_service_node ${typesupport_target})
target_link_libraries(go_to_pose_action_node ${typesupport_target})

install(TARGETS
	${PROJECT_NAME}_node
	${PROJECT_NAME}_with_service_node
    laser_scan_subscriber_node
	find_direction_buffers_node
	direction_service_node
	test_service_node
	go_to_pose_action_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()

