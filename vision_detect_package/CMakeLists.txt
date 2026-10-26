cmake_minimum_required(VERSION 3.17)
project(vision_detect_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(robosub_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# CUDA runtime (cudaMalloc, cudaMemcpy...) from JetPack's /usr/local/cuda
find_package(CUDAToolkit REQUIRED)

# TensorRT (installed by JetPack in the system folders, no CMake config file)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h REQUIRED)
find_library(NVINFER_LIB nvinfer REQUIRED)
find_library(NVONNXPARSER_LIB nvonnxparser REQUIRED)

add_executable(detect_node
  src/detect_node.cpp
  src/yolo_detector.cpp
)
target_include_directories(detect_node PRIVATE include)
# SYSTEM hides compiler warnings coming from inside the TensorRT headers
target_include_directories(detect_node SYSTEM PRIVATE
  ${TENSORRT_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(detect_node
  ${NVINFER_LIB}
  ${NVONNXPARSER_LIB}
  CUDA::cudart
  ${OpenCV_LIBS}
)
ament_target_dependencies(detect_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  robosub_msgs
)

install(TARGETS detect_node
  DESTINATION lib/${PROJECT_NAME}
)
# weights/ holds the .onnx model (and the .engine built from it)
install(DIRECTORY launch config weights
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
