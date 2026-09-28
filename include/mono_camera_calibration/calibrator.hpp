#ifndef MONO_CAMERA_CALIBRATION__CALIBRATOR_HPP_
#define MONO_CAMERA_CALIBRATION__CALIBRATOR_HPP_

#include <opencv2/core.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace mono_camera_calibration
{

struct CalibrationOptions
{
  cv::Size board_size{7, 7};
  double square_size_m{0.03};
  std::size_t maximum_samples{40};
  double minimum_sample_distance{0.12};
  int calibration_flags{0};
  bool asymmetric_grid{false};
};

struct CalibrationResult
{
  cv::Mat camera_matrix;
  cv::Mat distortion;
  std::vector<double> per_view_errors;
  double rms_error_px{0.0};
  double mean_reprojection_error_px{0.0};
  cv::Size image_size;
  std::size_t sample_count{0};

  [[nodiscard]] bool valid() const noexcept;
};

class MonoCalibrator
{
public:
  explicit MonoCalibrator(CalibrationOptions options);

  bool addSample(
    const std::vector<cv::Point2f> & image_points, const cv::Size & image_size,
    std::string & reason);
  [[nodiscard]] CalibrationResult solve() const;
  void clear();

  [[nodiscard]] std::size_t sampleCount() const noexcept {return image_points_.size();}
  [[nodiscard]] bool full() const noexcept;
  [[nodiscard]] const CalibrationOptions & options() const noexcept {return options_;}

private:
  [[nodiscard]] std::vector<cv::Point3f> objectPoints() const;
  [[nodiscard]] cv::Vec4d descriptor(
    const std::vector<cv::Point2f> & points, const cv::Size & image_size) const;

  CalibrationOptions options_;
  cv::Size image_size_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  std::vector<cv::Vec4d> descriptors_;
};

}  // namespace mono_camera_calibration

#endif  // MONO_CAMERA_CALIBRATION__CALIBRATOR_HPP_
