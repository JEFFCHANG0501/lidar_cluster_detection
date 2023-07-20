cmake_minimum_required(VERSION 3.5)
project(pcd_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(pcd_publisher SHARED
  src/pcd_publisher.cpp
)

ament_target_dependencies(pcd_publisher
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
)

add_executable(pcd_publisher_node
  src/pcd_publisher_node.cpp)

include_directories(
  include
)

target_link_libraries(pcd_publisher_node
  pcd_publisher
  Boost::filesystem
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
)

ament_target_dependencies(pcd_publisher_node
  rclcpp
)

install(TARGETS
  pcd_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pcd_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
