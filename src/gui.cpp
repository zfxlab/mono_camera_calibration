#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cctype>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace mono_camera_calibration
{

class CalibrationGui : public rclcpp::Node
{
public:
  CalibrationGui()
  : Node("mono_calibration_gui")
  {
    const std::string base = declare_parameter<std::string>("calibrator_namespace", "/mono_calibration");
    preview_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      base + "/preview/compressed", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        const cv::Mat encoded(1, static_cast<int>(message->data.size()), CV_8UC1,
          const_cast<unsigned char *>(message->data.data()));
        cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (!image.empty()) {
          std::lock_guard<std::mutex> lock(mutex_);
          image_ = std::move(image);
        }
      });
    status_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      base + "/status", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        if (message->status.empty()) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        status_ = message->status.front().message;
        values_.clear();
        for (const auto & value : message->status.front().values) {
          values_[value.key] = value.value;
        }
      });
    for (const std::string name : {"start", "stop", "reset", "calibrate", "save", "commit"}) {
      clients_[name] = create_client<std_srvs::srv::Trigger>(base + "/" + name);
    }
  }

  cv::Mat displayImage()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cv::Mat display = image_.empty() ? cv::Mat::zeros(480, 800, CV_8UC3) : image_.clone();
    const std::string details = "state=" + status_ + "  capture=" + value("capturing") +
      "  samples=" + value("accepted_samples") + "/" + value("minimum_samples") +
      "  RMS=" + value("rms_error_px");
    cv::rectangle(
      display, {0, display.rows - 102}, {display.cols, display.rows}, {24, 24, 24}, cv::FILLED);
    cv::putText(display, details, {12, display.rows - 76}, cv::FONT_HERSHEY_SIMPLEX,
      0.46, {220, 220, 220}, 1, cv::LINE_AA);
    cv::putText(
      display, "decision=" + value("last_decision"), {12, display.rows - 54},
      cv::FONT_HERSHEY_SIMPLEX, 0.43, {190, 210, 220}, 1, cv::LINE_AA);
    cv::putText(
      display, "action=" + action_status_, {12, display.rows - 32},
      cv::FONT_HERSHEY_SIMPLEX, 0.43, {170, 230, 170}, 1, cv::LINE_AA);
    cv::putText(display, "G start | X stop | R reset | C calibrate | S save | U commit | Q quit",
      {12, display.rows - 10}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {170, 210, 255}, 1, cv::LINE_AA);
    return display;
  }

  void call(const std::string & name)
  {
    const auto iterator = clients_.find(name);
    if (iterator == clients_.end() || !iterator->second->service_is_ready()) {
      setActionStatus(name + ": service not ready");
      RCLCPP_WARN(get_logger(), "service '%s' is not ready", name.c_str());
      return;
    }
    setActionStatus(name + ": request sent");
    iterator->second->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        setActionStatus(name + ": " + response->message);
        if (response->success) {
          RCLCPP_INFO(get_logger(), "%s: %s", name.c_str(), response->message.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "%s: %s", name.c_str(), response->message.c_str());
        }
      });
  }

private:
  void setActionStatus(const std::string & status)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    action_status_ = status;
  }

  std::string value(const std::string & key) const
  {
    const auto iterator = values_.find(key);
    return iterator == values_.end() ? "-" : iterator->second;
  }

  std::mutex mutex_;
  cv::Mat image_;
  std::string status_{"WAITING"};
  std::string action_status_{"ready"};
  std::map<std::string, std::string> values_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clients_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr preview_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_subscription_;
};

}  // namespace mono_camera_calibration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mono_camera_calibration::CalibrationGui>();
  constexpr char window[] = "C++ Monocular Camera Calibration";
  cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    cv::imshow(window, node->displayImage());
    const int key = std::tolower(
      static_cast<unsigned char>(cv::waitKey(10) & 0xff));
    if (key == 'q' || key == 27) {
      break;
    }
    if (key == 'g') {node->call("start");}
    if (key == 'x') {node->call("stop");}
    if (key == 'r') {node->call("reset");}
    if (key == 'c') {node->call("calibrate");}
    if (key == 's') {node->call("save");}
    if (key == 'u') {node->call("commit");}
  }
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
