cmake_minimum_required(VERSION 3.8)
project(sonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sonar_pub src/sonar_publisher.cpp)
ament_target_dependencies(sonar_pub rclcpp std_msgs)

add_executable(sonar_sub src/sonar_subscriber.cpp)
ament_target_dependencies(sonar_sub rclcpp std_msgs)

install(TARGETS
  sonar_pub
  sonar_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
