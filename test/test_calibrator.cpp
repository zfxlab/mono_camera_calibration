#include "mono_camera_calibration/calibrator.hpp"

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

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
    const cv::Vec3d rotation(0.03 * index, -0.02 * index, 0.01 * index);
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
