#include "mono_camera_calibration/calibrator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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
    !(options_.square_size_m > 0.0) || options_.minimum_samples < 3 ||
    options_.maximum_samples < options_.minimum_samples ||
    !(options_.minimum_sample_distance >= 0.0) ||
    !(options_.outlier_minimum_error_px >= 0.0) || !(options_.outlier_mad_scale >= 0.0) ||
    !(options_.minimum_pose_tilt_degrees > 0.0) ||
    options_.minimum_pose_tilt_degrees >= 90.0 ||
    options_.minimum_tilted_samples_per_direction == 0)
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

std::vector<cv::Point2f> MonoCalibrator::normalizedPoints(
  const std::vector<cv::Point2f> & points, const cv::Size & image_size) const
{
  std::vector<cv::Point2f> normalized;
  normalized.reserve(points.size());
  for (const auto & point : points) {
    normalized.emplace_back(
      point.x / static_cast<float>(image_size.width),
      point.y / static_cast<float>(image_size.height));
  }
  return normalized;
}

double MonoCalibrator::sampleDistance(
  const std::vector<cv::Point2f> & first,
  const std::vector<cv::Point2f> & second) const
{
  if (first.size() != second.size() || first.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const int rows = options_.board_size.height;
  const int columns = options_.board_size.width;
  const bool square = rows == columns && !options_.asymmetric_grid;
  const int transform_count = square ? 8 : 2;
  double best = std::numeric_limits<double>::infinity();
  for (int transform = 0; transform < transform_count; ++transform) {
    double squared_distance = 0.0;
    for (int row = 0; row < rows; ++row) {
      for (int column = 0; column < columns; ++column) {
        const std::size_t index = static_cast<std::size_t>(row * columns + column);
        int other_row = row;
        int other_column = column;
        if (square) {
          switch (transform) {
            case 1: other_row = columns - 1 - column; other_column = row; break;
            case 2: other_row = rows - 1 - row; other_column = columns - 1 - column; break;
            case 3: other_row = column; other_column = rows - 1 - row; break;
            case 4: other_column = columns - 1 - column; break;
            case 5: other_row = rows - 1 - row; break;
            case 6: other_row = column; other_column = row; break;
            case 7:
              other_row = columns - 1 - column;
              other_column = rows - 1 - row;
              break;
            default: break;
          }
        } else if (transform == 1) {
          other_row = rows - 1 - row;
          other_column = columns - 1 - column;
        }
        const auto difference = first[index] - second[
          static_cast<std::size_t>(other_row * columns + other_column)];
        squared_distance += difference.dot(difference);
      }
    }
    best = std::min(best, std::sqrt(squared_distance / first.size()));
  }
  return best;
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
  const auto candidate = normalizedPoints(image_points, image_size);
  const auto too_close = std::any_of(
    normalized_samples_.begin(), normalized_samples_.end(), [&](const auto & previous) {
      return sampleDistance(candidate, previous) < options_.minimum_sample_distance;
    });
  if (too_close) {
    reason = "pose_not_diverse";
    return false;
  }
  image_size_ = image_size;
  image_points_.push_back(image_points);
  normalized_samples_.push_back(candidate);
  reason = "accepted";
  return true;
}

CalibrationResult MonoCalibrator::solveSamples(
  const std::vector<std::vector<cv::Point2f>> & samples) const
{
  std::vector<std::vector<cv::Point3f>> object_points(samples.size(), objectPoints());
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat distortion;
  std::vector<cv::Mat> rotation_vectors;
  std::vector<cv::Mat> translation_vectors;
  const double rms = cv::calibrateCamera(
    object_points, samples, image_size_, camera_matrix, distortion,
    rotation_vectors, translation_vectors, options_.calibration_flags,
    cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-10));

  std::vector<double> view_errors;
  double total_squared_error = 0.0;
  std::size_t total_points = 0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(
      object_points[index], rotation_vectors[index], translation_vectors[index],
      camera_matrix, distortion, projected);
    const double squared_error =
      std::pow(cv::norm(samples[index], projected, cv::NORM_L2), 2);
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
  result.sample_count = samples.size();
  const double minimum_tilt_radians =
    options_.minimum_pose_tilt_degrees * CV_PI / 180.0;
  for (const auto & rotation_vector : rotation_vectors) {
    cv::Mat rotation;
    cv::Rodrigues(rotation_vector, rotation);
    const double normal_x = rotation.at<double>(0, 2);
    const double normal_y = rotation.at<double>(1, 2);
    const double normal_z = rotation.at<double>(2, 2);
    const double horizontal_tilt = std::atan2(normal_x, normal_z);
    const double vertical_tilt = std::atan2(normal_y, normal_z);
    result.tilted_left_count += horizontal_tilt <= -minimum_tilt_radians;
    result.tilted_right_count += horizontal_tilt >= minimum_tilt_radians;
    result.tilted_up_count += vertical_tilt <= -minimum_tilt_radians;
    result.tilted_down_count += vertical_tilt >= minimum_tilt_radians;
  }
  return result;
}

void MonoCalibrator::checkPoseCoverage(const CalibrationResult & result) const
{
  const std::size_t required = options_.minimum_tilted_samples_per_direction;
  if (result.tilted_left_count >= required && result.tilted_right_count >= required &&
    result.tilted_up_count >= required && result.tilted_down_count >= required)
  {
    return;
  }
  throw std::runtime_error(
          "insufficient pose coverage: need at least " + std::to_string(required) +
    " sample(s) tilted left/right/up/down by " +
    std::to_string(options_.minimum_pose_tilt_degrees) + " degrees; got " +
    std::to_string(result.tilted_left_count) + "/" +
    std::to_string(result.tilted_right_count) + "/" +
    std::to_string(result.tilted_up_count) + "/" +
    std::to_string(result.tilted_down_count));
}

CalibrationResult MonoCalibrator::solve() const
{
  if (image_points_.size() < options_.minimum_samples || image_size_.area() <= 0) {
    throw std::runtime_error("not enough calibration samples");
  }
  CalibrationResult initial = solveSamples(image_points_);
  initial.initial_sample_count = image_points_.size();

  std::vector<double> sorted_errors = initial.per_view_errors;
  const auto middle = sorted_errors.begin() + static_cast<std::ptrdiff_t>(sorted_errors.size() / 2);
  std::nth_element(sorted_errors.begin(), middle, sorted_errors.end());
  const double median_error = *middle;
  std::vector<double> deviations;
  deviations.reserve(initial.per_view_errors.size());
  for (const double error : initial.per_view_errors) {
    deviations.push_back(std::abs(error - median_error));
  }
  const auto deviation_middle =
    deviations.begin() + static_cast<std::ptrdiff_t>(deviations.size() / 2);
  std::nth_element(deviations.begin(), deviation_middle, deviations.end());
  constexpr double mad_to_sigma = 1.4826;
  const double threshold = std::max(
    options_.outlier_minimum_error_px,
    median_error + options_.outlier_mad_scale * mad_to_sigma * *deviation_middle);

  std::vector<std::size_t> outliers;
  for (std::size_t index = 0; index < initial.per_view_errors.size(); ++index) {
    if (initial.per_view_errors[index] > threshold) {
      outliers.push_back(index);
    }
  }
  std::sort(outliers.begin(), outliers.end(), [&](const auto left, const auto right) {
      return initial.per_view_errors[left] > initial.per_view_errors[right];
  });
  const std::size_t rejection_count = std::min(
    outliers.size(), image_points_.size() - options_.minimum_samples);
  if (rejection_count == 0) {
    checkPoseCoverage(initial);
    return initial;
  }

  std::vector<bool> rejected(image_points_.size(), false);
  for (std::size_t index = 0; index < rejection_count; ++index) {
    rejected[outliers[index]] = true;
  }
  std::vector<std::vector<cv::Point2f>> filtered;
  filtered.reserve(image_points_.size() - rejection_count);
  for (std::size_t index = 0; index < image_points_.size(); ++index) {
    if (!rejected[index]) {
      filtered.push_back(image_points_[index]);
    }
  }
  CalibrationResult result = solveSamples(filtered);
  result.initial_sample_count = image_points_.size();
  result.rejected_sample_count = rejection_count;
  checkPoseCoverage(result);
  return result;
}

void MonoCalibrator::clear()
{
  image_size_ = {};
  image_points_.clear();
  normalized_samples_.clear();
}

bool MonoCalibrator::full() const noexcept
{
  return image_points_.size() >= options_.maximum_samples;
}

}  // namespace mono_camera_calibration
