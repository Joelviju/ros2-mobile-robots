cmake_minimum_required(VERSION 3.8)
project(my_cpp_templates)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(basic_publisher src/basic_publisher.cpp)
ament_target_dependencies(basic_publisher rclcpp std_msgs)

add_executable(basic_subscriber src/basic_subscriber.cpp)
ament_target_dependencies(basic_subscriber rclcpp std_msgs)

install(TARGETS
  basic_publisher
  basic_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(velocity_publisher src/velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher rclcpp geometry_msgs)

add_executable(velocity_subscriber src/velocity_subscriber.cpp)
ament_target_dependencies(velocity_subscriber rclcpp geometry_msgs)

install(TARGETS
  velocity_publisher
  velocity_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(qos_publisher src/qos_publisher.cpp)
ament_target_dependencies(qos_publisher rclcpp std_msgs)

add_executable(qos_subscriber src/qos_subscriber.cpp)
ament_target_dependencies(qos_subscriber rclcpp std_msgs)

install(TARGETS
  qos_publisher
  qos_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(fake_camera_publisher src/fake_camera_publisher.cpp)
ament_target_dependencies(fake_camera_publisher
  rclcpp
  sensor_msgs
)

add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber
  rclcpp
  sensor_msgs
)

install(TARGETS
  fake_camera_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)



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

ament_package()
