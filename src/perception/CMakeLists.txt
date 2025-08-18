cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(capture_camera_pub src/capture_camera_pub.cpp)
add_executable(camera_subscriber src/camera_subscriber.cpp)

target_include_directories(capture_camera_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_include_directories(camera_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(capture_camera_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  capture_camera_pub
  "rclcpp"
  "OpenCV"
  "cv_bridge"
)

target_compile_features(camera_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  camera_subscriber
  "rclcpp"
  "OpenCV"
  "cv_bridge"
)

install(TARGETS
  capture_camera_pub
  camera_subscriber
  DESTINATION lib/${PROJECT_NAME})

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
