cmake_minimum_required(VERSION 3.10)
project(orca_zed_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ZED SDK (installed to /usr/local/zed) and the CUDA version it was built against
find_package(ZED 5 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)

add_executable(zed_publisher_node src/zed_publisher_node.cpp)

# SYSTEM hides compiler warnings coming from inside the ZED/CUDA headers
target_include_directories(zed_publisher_node SYSTEM PRIVATE
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)
target_link_directories(zed_publisher_node PRIVATE
  ${ZED_LIBRARY_DIR}
  ${CUDA_LIBRARY_DIRS}
)
target_link_libraries(zed_publisher_node
  ${ZED_LIBRARIES}
  ${CUDA_CUDA_LIBRARY}
  ${CUDA_CUDART_LIBRARY}
)
ament_target_dependencies(zed_publisher_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS zed_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
