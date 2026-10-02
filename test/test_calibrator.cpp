#include "mono_camera_calibration/calibrator.hpp"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <string>
#include <vector>

TEST(MonoCalibrator, RecoversSyntheticPinholeIntrinsics)
{
  mono_camera_calibration::CalibrationOptions options;
  options.board_size = {7, 6};
  options.square_size_m = 0.04;
  options.maximum_samples = 20;
  options.minimum_sample_distance = 0.0;
  mono_camera_calibration::MonoCalibrator calibrator(options);

  std::vector<cv::Point3f> object_points;
  for (int row = 0; row < options.board_size.height; ++row) {
    for (int column = 0; column < options.board_size.width; ++column) {
      object_points.emplace_back(column * 0.04F, row * 0.04F, 0.0F);
    }
  }
  const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 800.0, 0.0, 640.0, 0.0, 810.0, 480.0, 0.0, 0.0, 1.0);
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  for (int index = 0; index < 10; ++index) {
    const cv::Vec3d rotation(
      0.12 * ((index % 4) - 1.5),
      0.12 * (((index + 2) % 4) - 1.5), 0.01 * index);
    const cv::Vec3d translation(-0.12 + index * 0.025, -0.08 + index * 0.012, 1.0 + index * 0.04);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
      object_points, rotation, translation, camera_matrix, distortion, image_points);
    std::string reason;
    ASSERT_TRUE(calibrator.addSample(image_points, {1280, 960}, reason)) << reason;
  }

  const auto result = calibrator.solve();
  ASSERT_TRUE(result.valid());
  EXPECT_NEAR(result.camera_matrix.at<double>(0, 0), 800.0, 1.0);
  EXPECT_NEAR(result.camera_matrix.at<double>(1, 1), 810.0, 1.0);
  EXPECT_LT(result.mean_reprojection_error_px, 0.01);
}

TEST(MonoCalibrator, RejectsDuplicatePose)
{
  mono_camera_calibration::CalibrationOptions options;
  options.board_size = {3, 3};
  options.minimum_sample_distance = 0.1;
  mono_camera_calibration::MonoCalibrator calibrator(options);
  std::vector<cv::Point2f> points;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      points.emplace_back(200.0F + column * 20.0F, 150.0F + row * 20.0F);
    }
  }
  std::string reason;
  EXPECT_TRUE(calibrator.addSample(points, {640, 480}, reason));
  EXPECT_FALSE(calibrator.addSample(points, {640, 480}, reason));
  EXPECT_EQ(reason, "pose_not_diverse");
}

TEST(MonoCalibrator, RejectsHighReprojectionErrorView)
{
  mono_camera_calibration::CalibrationOptions options;
  options.board_size = {7, 6};
  options.square_size_m = 0.04;
  options.minimum_samples = 8;
  options.maximum_samples = 20;
  options.minimum_sample_distance = 0.0;
  options.outlier_minimum_error_px = 0.15;
  options.outlier_mad_scale = 3.0;
  mono_camera_calibration::MonoCalibrator calibrator(options);

  std::vector<cv::Point3f> object_points;
  for (int row = 0; row < options.board_size.height; ++row) {
    for (int column = 0; column < options.board_size.width; ++column) {
      object_points.emplace_back(column * 0.04F, row * 0.04F, 0.0F);
    }
  }
  const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 800.0, 0.0, 640.0, 0.0, 810.0, 480.0, 0.0, 0.0, 1.0);
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  for (int index = 0; index < 12; ++index) {
    const cv::Vec3d rotation(
      -0.20 + 0.04 * index, 0.20 * std::sin(0.7 * index), 0.02 * index);
    const cv::Vec3d translation(
      -0.14 + index * 0.022, -0.10 + (index % 4) * 0.045, 0.9 + index * 0.035);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
      object_points, rotation, translation, camera_matrix, distortion, image_points);
    if (index == 11) {
      for (std::size_t point = 0; point < image_points.size(); point += 3) {
        image_points[point] += cv::Point2f(4.0F, -3.0F);
      }
    }
    std::string reason;
    ASSERT_TRUE(calibrator.addSample(image_points, {1280, 960}, reason)) << reason;
  }

  const auto result = calibrator.solve();
  ASSERT_TRUE(result.valid());
  EXPECT_EQ(result.initial_sample_count, 12U);
  EXPECT_EQ(result.sample_count, 11U);
  EXPECT_EQ(result.rejected_sample_count, 1U);
  EXPECT_LT(result.mean_reprojection_error_px, 0.01);
}

TEST(MonoCalibrator, RejectsInsufficientPoseCoverage)
{
  mono_camera_calibration::CalibrationOptions options;
  options.board_size = {7, 6};
  options.square_size_m = 0.04;
  options.minimum_samples = 8;
  options.maximum_samples = 12;
  options.minimum_sample_distance = 0.0;
  mono_camera_calibration::MonoCalibrator calibrator(options);

  std::vector<cv::Point3f> object_points;
  for (int row = 0; row < options.board_size.height; ++row) {
    for (int column = 0; column < options.board_size.width; ++column) {
      object_points.emplace_back(column * 0.04F, row * 0.04F, 0.0F);
    }
  }
  const cv::Mat camera_matrix =
    (cv::Mat_<double>(3, 3) << 800.0, 0.0, 640.0, 0.0, 810.0, 480.0, 0.0, 0.0, 1.0);
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);
  for (int index = 0; index < 8; ++index) {
    const cv::Vec3d rotation(-0.12 + 0.035 * index, 0.24, 0.02 * index);
    const cv::Vec3d translation(-0.12 + 0.03 * index, -0.08 + 0.02 * index, 1.0);
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
      object_points, rotation, translation, camera_matrix, distortion, image_points);
    std::string reason;
    ASSERT_TRUE(calibrator.addSample(image_points, {1280, 960}, reason)) << reason;
  }

  try {
    static_cast<void>(calibrator.solve());
    FAIL() << "expected pose coverage rejection";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("insufficient pose coverage"), std::string::npos);
  }
}
