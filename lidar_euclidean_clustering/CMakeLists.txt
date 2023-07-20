cmake_minimum_required(VERSION 3.5)
project(lidar_euclidean_clustering)

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
find_package(rclcpp)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED)
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

add_library(euclidean_clustering
  src/euclidean_clustering.cpp)

ament_target_dependencies(euclidean_clustering
  rclcpp
  sensor_msgs
  visualization_msgs
  PCL
  pcl_conversions
)

add_executable(euclidean_clustering_node
  src/euclidean_clustering_node.cpp)
  
include_directories(include)

target_link_libraries(euclidean_clustering_node
  euclidean_clustering
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(euclidean_clustering_node
  rclcpp)

install(TARGETS
  euclidean_clustering_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  euclidean_clustering
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
