// =============================================================================
// YoloDetector: runs a YOLOv8 / YOLO11 ONNX model on the GPU with TensorRT.
//
// No ROS in here, just OpenCV images in and boxes out.
//
// Based on the official Stereolabs sample:
//   /usr/local/zed/samples/object detection/custom detector/cpp/
//     tensorrt_yolov5-v6-v8_onnx/src/yolo.cpp
// TensorRT 10 API checked against /usr/include/aarch64-linux-gnu/NvInfer*.h
// =============================================================================
#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace vision_detect
{

// One detected object, in pixels of the original image
struct Detection
{
  float x1, y1, x2, y2;  // top-left and bottom-right corners
  int class_id;          // index into the model's class list
  float confidence;      // 0 to 1
};

class YoloDetector
{
public:
  // TensorRT messages get sent here (so the ROS node can print them)
  using LogFn = std::function<void (nvinfer1::ILogger::Severity, const char *)>;

  explicit YoloDetector(LogFn log_fn);
  ~YoloDetector();

  // Turn an .onnx file into a TensorRT .engine file optimized for this GPU.
  // Slow (a few minutes), so it only needs to happen once per model.
  bool buildEngine(const std::string & onnx_path, const std::string & engine_path, bool fp16);

  // Load a .engine file and get ready to run
  bool loadEngine(const std::string & engine_path);

  // Run the model on one BGR image. Returns boxes above conf_threshold,
  // after removing duplicate overlapping boxes (NMS).
  std::vector<Detection> detect(const cv::Mat & bgr, float conf_threshold, float nms_threshold);

  int numClasses() const {return static_cast<int>(num_classes_);}
  int inputWidth() const {return input_w_;}
  int inputHeight() const {return input_h_;}

private:
  // TensorRT requires a logger object; this one forwards to log_fn_
  class Logger : public nvinfer1::ILogger
  {
public:
    explicit Logger(LogFn fn)
    : fn_(std::move(fn)) {}
    void log(Severity severity, const char * msg) noexcept override {fn_(severity, msg);}

private:
    LogFn fn_;
  };

  Logger logger_;

  // TensorRT objects (in TensorRT 10 they are freed with plain delete)
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  // Model input/output info, read from the engine
  std::string input_name_, output_name_;
  int input_w_ = 0, input_h_ = 0;
  size_t num_classes_ = 0;   // e.g. 52 for a playing card model
  size_t num_anchors_ = 0;   // number of candidate boxes, e.g. 8400 for 640x640

  // Memory for the model's input and output
  std::vector<float> host_input_, host_output_;  // on the CPU
  void * dev_input_ = nullptr;                   // on the GPU
  void * dev_output_ = nullptr;
  cudaStream_t stream_ = nullptr;
};

}  // namespace vision_detect
