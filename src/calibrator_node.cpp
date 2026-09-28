#include "mono_camera_calibration/calibrator.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mono_camera_calibration
{
namespace
{
using Trigger = std_srvs::srv::Trigger;

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string formatDouble(const double value, const int precision = 4)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

template<typename T>
void writeSequence(std::ostream & stream, const T * data, const std::size_t size)
{
  stream << '[';
  for (std::size_t index = 0; index < size; ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << std::setprecision(16) << data[index];
  }
  stream << ']';
}
}  // namespace

class MonoCalibrationNode : public rclcpp::Node
{
public:
  explicit MonoCalibrationNode(const rclcpp::NodeOptions & options)
  : Node("mono_calibrator", options)
  {
    const int board_columns = declare_parameter<int>("board.columns", 7);
    const int board_rows = declare_parameter<int>("board.rows", 7);
    pattern_ = declare_parameter<std::string>("board.pattern", "circles");
    const double square_size = declare_parameter<double>("board.square_size_m", 0.03);
    minimum_samples_ = static_cast<std::size_t>(
      declare_parameter<int>("sampling.minimum_samples", 12));
    const int maximum_samples = declare_parameter<int>("sampling.maximum_samples", 40);
    const double minimum_sample_distance =
      declare_parameter<double>("sampling.minimum_sample_distance", 0.12);
    capturing_ = declare_parameter<bool>("sampling.auto_start", true);
    const double processing_rate = declare_parameter<double>("sampling.processing_rate_hz", 15.0);
    preview_scale_ = declare_parameter<double>("display.preview_scale", 0.6);
    jpeg_quality_ = declare_parameter<int>("display.jpeg_quality", 80);
    output_path_ = declare_parameter<std::string>("output_path", "/tmp/mono_camera.yaml");
    camera_name_ = declare_parameter<std::string>("camera_name", "camera");
    const std::string image_topic =
      declare_parameter<std::string>("image_topic", "/camera/image_raw");
    const std::string set_camera_info_service = declare_parameter<std::string>(
      "set_camera_info_service", "/camera/set_camera_info");

    if (pattern_ != "chessboard" && pattern_ != "circles" && pattern_ != "acircles") {
      throw std::invalid_argument("board.pattern must be chessboard, circles, or acircles");
    }
    if (minimum_samples_ < 3 || maximum_samples < static_cast<int>(minimum_samples_) ||
      !(processing_rate > 0.0) || !(preview_scale_ > 0.0) || preview_scale_ > 1.0 ||
      jpeg_quality_ < 1 || jpeg_quality_ > 100)
    {
      throw std::invalid_argument("invalid sampling or display parameters");
    }

    CalibrationOptions calibration_options;
    calibration_options.board_size = {board_columns, board_rows};
    calibration_options.square_size_m = square_size;
    calibration_options.maximum_samples = static_cast<std::size_t>(maximum_samples);
    calibration_options.minimum_sample_distance = minimum_sample_distance;
    calibration_options.asymmetric_grid = pattern_ == "acircles";
    if (declare_parameter<bool>("calibration.fix_principal_point", false)) {
      calibration_options.calibration_flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
    }
    if (declare_parameter<bool>("calibration.zero_tangent_distortion", false)) {
      calibration_options.calibration_flags |= cv::CALIB_ZERO_TANGENT_DIST;
    }
    if (declare_parameter<bool>("calibration.fix_aspect_ratio", false)) {
      calibration_options.calibration_flags |= cv::CALIB_FIX_ASPECT_RATIO;
    }
    calibrator_ = std::make_unique<MonoCalibrator>(calibration_options);
    state_ = capturing_ ? "WAITING_FOR_BOARD" : "PREVIEW_ONLY";

    preview_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "preview/compressed", rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("status", 10);
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_image_ = std::move(message);
        ++received_frames_;
      });
    set_camera_info_client_ =
      create_client<sensor_msgs::srv::SetCameraInfo>(set_camera_info_service);
    processing_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / processing_rate),
      std::bind(&MonoCalibrationNode::processLatestImage, this));
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&MonoCalibrationNode::publishStatus, this));

    start_service_ = createTriggerService("start", [this](std::string & message) {
      std::lock_guard<std::mutex> lock(mutex_);
      capturing_ = true;
      state_ = "WAITING_FOR_BOARD";
      message = "sample capture started";
      return true;
    });
    stop_service_ = createTriggerService("stop", [this](std::string & message) {
      std::lock_guard<std::mutex> lock(mutex_);
      capturing_ = false;
      state_ = "PREVIEW_ONLY";
      message = "sample capture stopped";
      return true;
    });
    reset_service_ = createTriggerService("reset", [this](std::string & message) {
      std::lock_guard<std::mutex> lock(mutex_);
      calibrator_->clear();
      result_ = {};
      state_ = capturing_ ? "WAITING_FOR_BOARD" : "PREVIEW_ONLY";
      last_decision_ = "reset";
      message = "samples and result cleared";
      return true;
    });
    calibrate_service_ = createTriggerService("calibrate", [this](std::string & message) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (calibrator_->sampleCount() < minimum_samples_) {
        message = "not enough diverse samples";
        return false;
      }
      try {
        state_ = "CALIBRATING";
        result_ = calibrator_->solve();
        state_ = "CALIBRATED";
        message = "calibration complete; RMS=" + formatDouble(result_.rms_error_px) + " px";
        return true;
      } catch (const std::exception & error) {
        state_ = "CALIBRATION_FAILED";
        message = error.what();
        return false;
      }
    });
    save_service_ = createTriggerService("save", [this](std::string & message) {
      std::lock_guard<std::mutex> lock(mutex_);
      return saveResult(message);
    });
    commit_service_ = createTriggerService("commit", [this](std::string & message) {
      return commitResult(message);
    });

    RCLCPP_INFO(
      get_logger(), "C++ mono calibrator ready: pattern=%s board=%dx%d image=%s",
      pattern_.c_str(), board_columns, board_rows, image_topic.c_str());
  }

private:
  using TriggerFunction = std::function<bool(std::string &)>;

  rclcpp::Service<Trigger>::SharedPtr createTriggerService(
    const std::string & name, TriggerFunction function)
  {
    return create_service<Trigger>(
      name, [function = std::move(function)](
        const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
        response->success = function(response->message);
      });
  }

  bool detectBoard(const cv::Mat & gray, std::vector<cv::Point2f> & points) const
  {
    const auto board_size = calibrator_->options().board_size;
    if (pattern_ == "chessboard") {
      const bool found = cv::findChessboardCorners(
        gray, board_size, points,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
      if (found) {
        cv::cornerSubPix(
          gray, points, {5, 5}, {-1, -1},
          cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.01));
      }
      return found;
    }
    const int flags = pattern_ == "acircles" ?
      cv::CALIB_CB_ASYMMETRIC_GRID : cv::CALIB_CB_SYMMETRIC_GRID;
    return cv::findCirclesGrid(gray, board_size, points, flags);
  }

  void processLatestImage()
  {
    sensor_msgs::msg::Image::ConstSharedPtr message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message = latest_image_;
      if (!message || message == last_processed_image_) {
        return;
      }
      last_processed_image_ = message;
    }

    cv::Mat gray;
    try {
      gray = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::MONO8)->image;
    } catch (const cv_bridge::Exception & error) {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = "IMAGE_CONVERSION_ERROR";
      last_decision_ = error.what();
      return;
    }

    std::vector<cv::Point2f> points;
    const bool found = detectBoard(gray, points);
    bool capturing = false;
    std::size_t sample_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++processed_frames_;
      capturing = capturing_;
      if (!found) {
        state_ = capturing_ ? "WAITING_FOR_BOARD" : "PREVIEW_ONLY";
        last_decision_ = "board_not_found";
      } else if (capturing_) {
        const bool accepted = calibrator_->addSample(points, gray.size(), last_decision_);
        state_ = accepted ?
          (calibrator_->full() ? "SAMPLE_LIMIT_REACHED" : "CAPTURING") : "BOARD_DETECTED";
      } else {
        state_ = "PREVIEW_ONLY";
        last_decision_ = "capture_stopped";
      }
      sample_count = calibrator_->sampleCount();
      frame_id_ = message->header.frame_id;
    }

    cv::Mat preview;
    cv::cvtColor(gray, preview, cv::COLOR_GRAY2BGR);
    if (found) {
      cv::drawChessboardCorners(preview, calibrator_->options().board_size, points, true);
    }
    const std::string summary =
      std::string(found ? "FOUND" : "SEARCHING") + "  samples " +
      std::to_string(sample_count) + "/" + std::to_string(minimum_samples_) +
      (capturing ? "  CAPTURE" : "  PAUSED");
    cv::putText(
      preview, summary, {16, 32}, cv::FONT_HERSHEY_SIMPLEX, 0.75,
      found ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
    if (preview_scale_ != 1.0) {
      cv::resize(preview, preview, {}, preview_scale_, preview_scale_, cv::INTER_AREA);
    }

    sensor_msgs::msg::CompressedImage output;
    output.header = message->header;
    output.format = "jpeg";
    cv::imencode(".jpg", preview, output.data, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    preview_publisher_->publish(std::move(output));
  }

  sensor_msgs::msg::CameraInfo cameraInfoFromResult()
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = now();
    info.header.frame_id = frame_id_;
    info.width = static_cast<std::uint32_t>(result_.image_size.width);
    info.height = static_cast<std::uint32_t>(result_.image_size.height);
    info.distortion_model = "plumb_bob";
    info.d.assign(
      result_.distortion.ptr<double>(),
      result_.distortion.ptr<double>() + result_.distortion.total());
    for (std::size_t index = 0; index < info.k.size(); ++index) {
      info.k[index] = result_.camera_matrix.at<double>(
        static_cast<int>(index / 3), static_cast<int>(index % 3));
    }
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {
      info.k[0], info.k[1], info.k[2], 0.0,
      info.k[3], info.k[4], info.k[5], 0.0,
      info.k[6], info.k[7], info.k[8], 0.0};
    return info;
  }

  bool saveResult(std::string & message)
  {
    if (!result_.valid()) {
      message = "no calibration result; run calibrate first";
      return false;
    }
    const auto info = cameraInfoFromResult();
    std::ofstream output(output_path_);
    if (!output) {
      message = "cannot open " + output_path_;
      return false;
    }
    output << "image_width: " << info.width << '\n';
    output << "image_height: " << info.height << '\n';
    output << "camera_name: " << camera_name_ << '\n';
    output << "camera_matrix:\n  rows: 3\n  cols: 3\n  data: ";
    writeSequence(output, info.k.data(), info.k.size());
    output << "\ndistortion_model: plumb_bob\ndistortion_coefficients:\n  rows: 1\n  cols: " <<
      info.d.size() << "\n  data: ";
    writeSequence(output, info.d.data(), info.d.size());
    output << "\nrectification_matrix:\n  rows: 3\n  cols: 3\n  data: ";
    writeSequence(output, info.r.data(), info.r.size());
    output << "\nprojection_matrix:\n  rows: 3\n  cols: 4\n  data: ";
    writeSequence(output, info.p.data(), info.p.size());
    output << '\n';
    if (!output) {
      message = "failed while writing " + output_path_;
      return false;
    }
    message = "saved " + output_path_;
    return true;
  }

  bool commitResult(std::string & message)
  {
    sensor_msgs::msg::CameraInfo info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!result_.valid()) {
        message = "no calibration result; run calibrate first";
        return false;
      }
      info = cameraInfoFromResult();
    }
    if (!set_camera_info_client_->service_is_ready()) {
      message = "set_camera_info service is not ready";
      return false;
    }
    auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
    request->camera_info = std::move(info);
    set_camera_info_client_->async_send_request(
      request, [this](rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedFuture future) {
        const auto response = future.get();
        std::lock_guard<std::mutex> lock(mutex_);
        last_decision_ = response->success ? "commit_succeeded" :
          "commit_failed: " + response->status_message;
      });
    message = "set_camera_info request submitted";
    return true;
  }

  void publishStatus()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string(": calibration");
    status.hardware_id = camera_name_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.level = state_ == "CALIBRATION_FAILED" || state_ == "IMAGE_CONVERSION_ERROR" ?
        diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = state_;
      status.values.push_back(keyValue("state", state_));
      status.values.push_back(keyValue("last_decision", last_decision_));
      status.values.push_back(keyValue("received_frames", std::to_string(received_frames_.load())));
      status.values.push_back(keyValue("processed_frames", std::to_string(processed_frames_.load())));
      status.values.push_back(keyValue("accepted_samples", std::to_string(calibrator_->sampleCount())));
      status.values.push_back(keyValue("minimum_samples", std::to_string(minimum_samples_)));
      status.values.push_back(keyValue("capturing", capturing_ ? "true" : "false"));
      status.values.push_back(keyValue(
        "rms_error_px", result_.valid() ? formatDouble(result_.rms_error_px) : "n/a"));
    }
    array.status.push_back(std::move(status));
    status_publisher_->publish(std::move(array));
  }

  std::mutex mutex_;
  std::unique_ptr<MonoCalibrator> calibrator_;
  CalibrationResult result_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  sensor_msgs::msg::Image::ConstSharedPtr last_processed_image_;
  std::atomic<std::uint64_t> received_frames_{0};
  std::atomic<std::uint64_t> processed_frames_{0};
  std::size_t minimum_samples_{12};
  bool capturing_{true};
  double preview_scale_{0.6};
  int jpeg_quality_{80};
  std::string pattern_;
  std::string state_;
  std::string last_decision_{"waiting_for_image"};
  std::string output_path_;
  std::string camera_name_;
  std::string frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
  rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_client_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::Service<Trigger>::SharedPtr calibrate_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Service<Trigger>::SharedPtr commit_service_;
};

}  // namespace mono_camera_calibration

RCLCPP_COMPONENTS_REGISTER_NODE(mono_camera_calibration::MonoCalibrationNode)
