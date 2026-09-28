#include "mono_camera_calibration/calibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace mono_camera_calibration
{

bool CalibrationResult::valid() const noexcept
{
  return !camera_matrix.empty() && !distortion.empty() && image_size.area() > 0 && sample_count > 0;
}

MonoCalibrator::MonoCalibrator(CalibrationOptions options)
: options_(std::move(options))
{
  if (options_.board_size.width < 2 || options_.board_size.height < 2 ||
    !(options_.square_size_m > 0.0) || options_.maximum_samples < 3 ||
    !(options_.minimum_sample_distance >= 0.0))
  {
    throw std::invalid_argument("invalid monocular calibration options");
  }
}

std::vector<cv::Point3f> MonoCalibrator::objectPoints() const
{
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<std::size_t>(options_.board_size.area()));
  for (int row = 0; row < options_.board_size.height; ++row) {
    for (int column = 0; column < options_.board_size.width; ++column) {
      const double x = options_.asymmetric_grid ?
        (2.0 * column + static_cast<double>(row % 2)) * options_.square_size_m :
        column * options_.square_size_m;
      points.emplace_back(
        static_cast<float>(x), static_cast<float>(row * options_.square_size_m), 0.0F);
    }
  }
  return points;
}

cv::Vec4d MonoCalibrator::descriptor(
  const std::vector<cv::Point2f> & points, const cv::Size & image_size) const
{
  const cv::Rect box = cv::boundingRect(points);
  const double width = static_cast<double>(image_size.width);
  const double height = static_cast<double>(image_size.height);
  const double center_x = (box.x + box.width * 0.5) / width;
  const double center_y = (box.y + box.height * 0.5) / height;
  const double scale = std::sqrt(static_cast<double>(box.area()) / (width * height));
  const auto & top_left = points.front();
  const auto & top_right = points[static_cast<std::size_t>(options_.board_size.width - 1)];
  const auto & bottom_left = points[
    static_cast<std::size_t>((options_.board_size.height - 1) * options_.board_size.width)];
  const double top = cv::norm(top_right - top_left);
  const double left = cv::norm(bottom_left - top_left);
  const double skew = (top + left) > 0.0 ? std::abs(top - left) / (top + left) : 0.0;
  return {center_x, center_y, scale, skew};
}

bool MonoCalibrator::addSample(
  const std::vector<cv::Point2f> & image_points, const cv::Size & image_size,
  std::string & reason)
{
  if (full()) {
    reason = "sample_limit_reached";
    return false;
  }
  if (image_size.area() <= 0 ||
    image_points.size() != static_cast<std::size_t>(options_.board_size.area()))
  {
    reason = "invalid_detection";
    return false;
  }
  if (image_size_.area() > 0 && image_size != image_size_) {
    reason = "image_size_changed";
    return false;
  }
  const cv::Vec4d candidate = descriptor(image_points, image_size);
  const auto too_close = std::any_of(
    descriptors_.begin(), descriptors_.end(), [&](const cv::Vec4d & previous) {
      return cv::norm(candidate - previous) < options_.minimum_sample_distance;
    });
  if (too_close) {
    reason = "pose_not_diverse";
    return false;
  }
  image_size_ = image_size;
  image_points_.push_back(image_points);
  descriptors_.push_back(candidate);
  reason = "accepted";
  return true;
}

CalibrationResult MonoCalibrator::solve() const
{
  if (image_points_.size() < 3 || image_size_.area() <= 0) {
    throw std::runtime_error("at least three calibration samples are required");
  }
  std::vector<std::vector<cv::Point3f>> object_points(image_points_.size(), objectPoints());
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion;
  std::vector<cv::Mat> rotation_vectors;
  std::vector<cv::Mat> translation_vectors;
  const double rms = cv::calibrateCamera(
    object_points, image_points_, image_size_, camera_matrix, distortion,
    rotation_vectors, translation_vectors, options_.calibration_flags);

  std::vector<double> view_errors;
  double total_squared_error = 0.0;
  std::size_t total_points = 0;
  for (std::size_t index = 0; index < image_points_.size(); ++index) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(
      object_points[index], rotation_vectors[index], translation_vectors[index],
      camera_matrix, distortion, projected);
    const double squared_error =
      std::pow(cv::norm(image_points_[index], projected, cv::NORM_L2), 2);
    view_errors.push_back(std::sqrt(squared_error / projected.size()));
    total_squared_error += squared_error;
    total_points += projected.size();
  }

  CalibrationResult result;
  result.camera_matrix = camera_matrix;
  result.distortion = distortion;
  result.per_view_errors = std::move(view_errors);
  result.rms_error_px = rms;
  result.mean_reprojection_error_px = std::sqrt(total_squared_error / total_points);
  result.image_size = image_size_;
  result.sample_count = image_points_.size();
  return result;
}

void MonoCalibrator::clear()
{
  image_size_ = {};
  image_points_.clear();
  descriptors_.clear();
}

bool MonoCalibrator::full() const noexcept
{
  return image_points_.size() >= options_.maximum_samples;
}

}  // namespace mono_camera_calibration
