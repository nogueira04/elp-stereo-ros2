cmake_minimum_required(VERSION 3.8)
project(elp_stereo)

# Compiler options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(split_sync_image_node src/split_sync.cpp)

ament_target_dependencies(split_sync_image_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  camera_info_manager
)

install(TARGETS split_sync_image_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY calibration
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  
  set(ament_cmake_cpplint_FOUND TRUE)    
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

