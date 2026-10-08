cmake_minimum_required(VERSION 3.20)

project(carla_gnss_processing
  VERSION 0.1.0
  DESCRIPTION "Combines two CARLA GNSS sensors into one dual-antenna reading (position and heading)"
  LANGUAGES C CXX)

# Default to a Release build if the user doesn't specify one
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type specified, defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sad_msgs REQUIRED)

list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

# GNSS processing node: two CARLA GNSS sensors -> one sad_msgs/Gnss with dual-antenna heading
add_executable(carla_gnss_processing_node
  src/carla_gnss_processing_node.cpp
  src/carla_gnss_processing.cpp
  src/map_projection.cpp)

target_include_directories(carla_gnss_processing_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GeographicLib_INCLUDE_DIRS})

target_link_libraries(carla_gnss_processing_node
  PRIVATE
    rclcpp::rclcpp
    message_filters::message_filters
    ${tf2_ros_TARGETS}
    ${tf2_eigen_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    Eigen3::Eigen
    ${sad_msgs_TARGETS}
    ${GeographicLib_LIBRARIES})

target_compile_features(carla_gnss_processing_node PRIVATE cxx_std_20)

target_compile_options(carla_gnss_processing_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic)

install(TARGETS carla_gnss_processing_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

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
