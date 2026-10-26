// =============================================================================
// YoloDetector implementation. See yolo_detector.hpp for the overview.
//
// Engine build / load and output decoding follow the official Stereolabs
// sample (tensorrt_yolov5-v6-v8_onnx/src/yolo.cpp). The letterbox
// preprocessing matches what Ultralytics does (gray 114 padding, centered).
// =============================================================================
#include "vision_detect_package/yolo_detector.hpp"

#include <NvOnnxParser.h>

#include <algorithm>
#include <fstream>
#include <iterator>

#include <opencv2/imgproc.hpp>

namespace vision_detect
{

namespace
{

// Print a readable message if a CUDA call fails
bool cudaOk(cudaError_t err, const char * what, nvinfer1::ILogger & log)
{
  if (err == cudaSuccess) {
    return true;
  }
  std::string msg = std::string(what) + " failed: " + cudaGetErrorString(err);
  log.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
  return false;
}

// How much two boxes overlap: 0 = not at all, 1 = identical
float iou(const Detection & a, const Detection & b)
{
  float w = std::max(0.f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
  float h = std::max(0.f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
  float overlap = w * h;
  float total = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - overlap;
  return total > 0.f ? overlap / total : 0.f;
}

}  // namespace

YoloDetector::YoloDetector(LogFn log_fn)
: logger_(log_fn) {}

YoloDetector::~YoloDetector()
{
  // Free GPU memory (the TensorRT objects free themselves via unique_ptr)
  if (dev_input_) {cudaFree(dev_input_);}
  if (dev_output_) {cudaFree(dev_output_);}
  if (stream_) {cudaStreamDestroy(stream_);}
}

// -----------------------------------------------------------------------------
// ONNX -> TensorRT engine (same steps as Yolo::build_engine in the ZED sample)
// -----------------------------------------------------------------------------
bool YoloDetector::buildEngine(
  const std::string & onnx_path, const std::string & engine_path, bool fp16)
{
  using Severity = nvinfer1::ILogger::Severity;

  std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger_));
  if (!builder) {return false;}
  // 0 = default flags (TensorRT 10 always uses "explicit batch")
  std::unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(0));
  std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
  std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, logger_));
  if (!network || !config || !parser) {return false;}

  // Read the model layers from the .onnx file
  if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(Severity::kWARNING))) {
    logger_.log(Severity::kERROR, ("Could not parse " + onnx_path).c_str());
    return false;
  }

  // Half precision (fp16): about 2x faster on Jetson with almost no accuracy loss
  if (fp16) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  // The slow part: TensorRT tries many ways to run each layer and keeps the fastest
  std::unique_ptr<nvinfer1::IHostMemory> plan(builder->buildSerializedNetwork(*network, *config));
  if (!plan || plan->size() == 0) {
    logger_.log(Severity::kERROR, "TensorRT engine build failed");
    return false;
  }

  // Save it so next time we can skip straight to loadEngine()
  std::ofstream out(engine_path, std::ios::binary);
  out.write(static_cast<const char *>(plan->data()), plan->size());
  return out.good();
}

// -----------------------------------------------------------------------------
// Load a saved engine (same steps as Yolo::init in the ZED sample)
// -----------------------------------------------------------------------------
bool YoloDetector::loadEngine(const std::string & engine_path)
{
  using Severity = nvinfer1::ILogger::Severity;

  // Read the whole file into memory
  std::ifstream file(engine_path, std::ios::binary);
  if (!file) {return false;}
  std::vector<char> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {return false;}
  engine_.reset(runtime_->deserializeCudaEngine(data.data(), data.size()));
  if (!engine_) {return false;}
  context_.reset(engine_->createExecutionContext());
  if (!context_) {return false;}

  // A YOLO detection model has exactly one input (the image) and one output
  if (engine_->getNbIOTensors() != 2) {
    logger_.log(Severity::kERROR, "Expected a model with 1 input and 1 output");
    return false;
  }
  for (int i = 0; i < 2; ++i) {
    const char * name = engine_->getIOTensorName(i);
    nvinfer1::Dims d = engine_->getTensorShape(name);
    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      // Input shape: 1 x 3 x height x width
      input_name_ = name;
      input_h_ = d.d[2];
      input_w_ = d.d[3];
    } else {
      // Output shape for YOLOv8 / YOLO11: 1 x (4 + classes) x anchors
      // (4 = box center x, center y, width, height)
      output_name_ = name;
      if (d.d[1] <= 4 || d.d[1] > d.d[2]) {
        logger_.log(Severity::kERROR,
          "Output is not YOLOv8/YOLO11 style (1 x (4+classes) x anchors)");
        return false;
      }
      num_classes_ = d.d[1] - 4;
      num_anchors_ = d.d[2];
    }
  }
  if (input_w_ <= 0 || input_h_ <= 0) {
    logger_.log(Severity::kERROR, "Model has a dynamic input size; export with dynamic=False");
    return false;
  }

  // Allocate input/output memory on the CPU and GPU
  host_input_.resize(3 * input_w_ * input_h_);
  host_output_.resize((4 + num_classes_) * num_anchors_);
  return cudaOk(cudaMalloc(&dev_input_, host_input_.size() * sizeof(float)), "cudaMalloc", logger_) &&
         cudaOk(cudaMalloc(&dev_output_, host_output_.size() * sizeof(float)), "cudaMalloc", logger_) &&
         cudaOk(cudaStreamCreate(&stream_), "cudaStreamCreate", logger_);
}

// -----------------------------------------------------------------------------
// Run the model on one image
// -----------------------------------------------------------------------------
std::vector<Detection> YoloDetector::detect(
  const cv::Mat & bgr, float conf_threshold, float nms_threshold)
{
  std::vector<Detection> found;
  if (!context_) {return found;}

  // 1) Letterbox: shrink the image to fit the model input without stretching
  //    it, and fill the leftover space with gray (114, like Ultralytics)
  const float scale = std::min(
    static_cast<float>(input_w_) / bgr.cols, static_cast<float>(input_h_) / bgr.rows);
  const int new_w = static_cast<int>(bgr.cols * scale + 0.5f);
  const int new_h = static_cast<int>(bgr.rows * scale + 0.5f);
  const int pad_x = (input_w_ - new_w) / 2;  // padding on the left
  const int pad_y = (input_h_ - new_h) / 2;  // padding on the top
  cv::Mat resized, boxed;
  cv::resize(bgr, resized, cv::Size(new_w, new_h));
  cv::copyMakeBorder(resized, boxed, pad_y, input_h_ - new_h - pad_y,
    pad_x, input_w_ - new_w - pad_x, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

  // 2) BGR -> RGB, pixels 0-255 -> 0-1, and reorder to 3 separate color planes
  //    (the model wants all red values, then all green, then all blue)
  cv::Mat rgb;
  cv::cvtColor(boxed, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
  const int plane = input_w_ * input_h_;
  std::vector<cv::Mat> planes = {
    cv::Mat(input_h_, input_w_, CV_32F, host_input_.data()),
    cv::Mat(input_h_, input_w_, CV_32F, host_input_.data() + plane),
    cv::Mat(input_h_, input_w_, CV_32F, host_input_.data() + 2 * plane)};
  cv::split(rgb, planes);  // writes straight into host_input_

  // 3) Copy to the GPU, run the model, copy the answer back
  cudaMemcpyAsync(dev_input_, host_input_.data(), host_input_.size() * sizeof(float),
    cudaMemcpyHostToDevice, stream_);
  context_->setTensorAddress(input_name_.c_str(), dev_input_);
  context_->setTensorAddress(output_name_.c_str(), dev_output_);
  if (!context_->enqueueV3(stream_)) {
    logger_.log(nvinfer1::ILogger::Severity::kERROR, "TensorRT inference failed");
    return found;
  }
  cudaMemcpyAsync(host_output_.data(), dev_output_, host_output_.size() * sizeof(float),
    cudaMemcpyDeviceToHost, stream_);
  if (!cudaOk(cudaStreamSynchronize(stream_), "inference", logger_)) {return found;}

  // 4) Decode. The output is stored row by row:
  //    row 0 = all center x's, row 1 = all center y's, row 2 = widths,
  //    row 3 = heights, rows 4.. = the score of each class for every anchor
  const size_t n = num_anchors_;
  const float * out = host_output_.data();
  std::vector<Detection> candidates;
  for (size_t i = 0; i < n; ++i) {
    // Pick the class with the highest score for this anchor
    int best_class = 0;
    float best_score = 0.f;
    for (size_t c = 0; c < num_classes_; ++c) {
      float s = out[(4 + c) * n + i];
      if (s > best_score) {
        best_score = s;
        best_class = static_cast<int>(c);
      }
    }
    if (best_score < conf_threshold) {continue;}

    // Undo the letterbox: remove padding, then scale back to the original size
    float cx = (out[0 * n + i] - pad_x) / scale;
    float cy = (out[1 * n + i] - pad_y) / scale;
    float w = out[2 * n + i] / scale;
    float h = out[3 * n + i] / scale;
    Detection d;
    d.x1 = std::clamp(cx - w / 2, 0.f, static_cast<float>(bgr.cols));
    d.y1 = std::clamp(cy - h / 2, 0.f, static_cast<float>(bgr.rows));
    d.x2 = std::clamp(cx + w / 2, 0.f, static_cast<float>(bgr.cols));
    d.y2 = std::clamp(cy + h / 2, 0.f, static_cast<float>(bgr.rows));
    d.class_id = best_class;
    d.confidence = best_score;
    candidates.push_back(d);
  }

  // 5) NMS: the model finds the same object several times with slightly
  //    different boxes. Keep the most confident box and drop any box of the
  //    same class that overlaps it too much.
  std::sort(candidates.begin(), candidates.end(),
    [](const Detection & a, const Detection & b) {return a.confidence > b.confidence;});
  for (const auto & cand : candidates) {
    bool keep = true;
    for (const auto & kept : found) {
      if (kept.class_id == cand.class_id && iou(kept, cand) > nms_threshold) {
        keep = false;
        break;
      }
    }
    if (keep) {found.push_back(cand);}
  }
  return found;
}

}  // namespace vision_detect
