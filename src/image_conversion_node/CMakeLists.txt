cmake_minimum_required(VERSION 3.8)
project(image_conversion_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(services)

add_executable(image_service_node src/image_conversion_node.cpp)
ament_target_dependencies(image_service_node rclcpp sensor_msgs cv_bridge OpenCV services)


install(TARGETS
  image_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
