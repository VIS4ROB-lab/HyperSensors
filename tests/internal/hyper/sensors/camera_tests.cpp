/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <string>

#include <gtest/gtest.h>

#include "hyper/sensors/camera.hpp"
#include "hyper/utils.hpp"

namespace hyper::sensors::tests {

class CameraTests : public testing::Test {
 protected:
  // Constants.
  static constexpr auto kFilePath = __FILE__;
  static constexpr auto kInputFileName = "camera.yaml";
  static constexpr auto kOutputFileName = "camera_out.yaml";

  static constexpr auto kNumIterations = 25;
  static constexpr auto kMinLandmarkDistance = 1.0;
  static constexpr auto kMaxLandmarkDistance = 10.0;
  static constexpr auto kNumericIncrement = 1e-6;
  static constexpr auto kNumericTolerance = 1e-6;

  // Definitions.
  using Path = std::filesystem::path;

  using Scalar = double;
  using Camera = sensors::Camera;
  using Pixel = Camera::Pixel;
  using Bearing = Camera::Bearing;
  using Landmark = Camera::Landmark;
  using BearingPixelJacobian = variables::JacobianNM<Bearing, Pixel>;
  using PixelLandmarkJacobian = variables::JacobianNM<Pixel, Landmark>;
  using BearingLandmarkJacobian = variables::JacobianNM<Bearing, Landmark>;

  /// Setup.
  auto SetUp() -> void final {
    input_ = Path{kFilePath}.parent_path() /= kInputFileName;
    YAML::LoadFile(input_) >> camera_;
  }

  Path input_;
  Camera camera_;
};

TEST_F(CameraTests, ReadWrite) {
  YAML::Emitter emitter;
  const auto output = Path{kFilePath}.parent_path() /= kOutputFileName;
  std::ofstream{output} << (emitter << camera_).c_str();
  EXPECT_TRUE(internal::CompareFiles(input_, output));
  std::remove(output.c_str());
}

TEST_F(CameraTests, Duality) {
  for (auto i = 0; i < kNumIterations; ++i) {
    const Landmark landmark = Eigen::internal::random<Scalar>(kMinLandmarkDistance, kMaxLandmarkDistance) * camera_.randomBearing();
    const auto pixel = Camera::LandmarkToPixel(landmark);
    const auto bearing_a = Camera::LandmarkToBearing(landmark);
    const auto bearing_b = Camera::PixelToBearing(pixel);
    EXPECT_TRUE(bearing_a.isApprox(bearing_b));
  }
}

TEST_F(CameraTests, LandmarkToPixelJacobian) {
  for (auto i = 0; i < kNumIterations; ++i) {
    const Landmark landmark = Eigen::internal::random<Scalar>(kMinLandmarkDistance, kMaxLandmarkDistance) * camera_.randomBearing();

    PixelLandmarkJacobian J_a, J_n;
    const auto pixel = Camera::LandmarkToPixel(landmark, J_a.data());

    for (auto j = 0; j < Landmark::kNumParameters; ++j) {
      const auto d_pixel = Camera::LandmarkToPixel(landmark + kNumericIncrement * Landmark::Unit(j));
      J_n.col(j) = (d_pixel - pixel) / kNumericIncrement;
    }

    EXPECT_TRUE(J_n.isApprox(J_a, kNumericTolerance));
  }
}

TEST_F(CameraTests, LandmarkToBearingJacobian) {
  for (auto i = 0; i < kNumIterations; ++i) {
    const Landmark landmark = Eigen::internal::random<Scalar>(kMinLandmarkDistance, kMaxLandmarkDistance) * camera_.randomBearing();

    BearingLandmarkJacobian J_a, J_n;
    const auto bearing = Camera::LandmarkToBearing(landmark, J_a.data());

    for (auto j = 0; j < Landmark::kNumParameters; ++j) {
      const auto d_bearing = Camera::LandmarkToBearing(landmark + kNumericIncrement * Landmark::Unit(j));
      J_n.col(j) = (d_bearing - bearing) / kNumericIncrement;
    }

    EXPECT_TRUE(J_n.isApprox(J_a, kNumericTolerance));
  }
}

TEST_F(CameraTests, PixelToBearingJacobian) {
  for (auto i = 0; i < kNumIterations; ++i) {
    const auto normalized_pixel = camera_.randomNormalizedPixel();

    BearingPixelJacobian J_a, J_n;
    const auto bearing = Camera::PixelToBearing(normalized_pixel, J_a.data());

    for (auto j = 0; j < Pixel::kNumParameters; ++j) {
      const auto d_bearing = Camera::PixelToBearing(normalized_pixel + kNumericIncrement * Pixel::Unit(j));
      J_n.col(j) = (d_bearing - bearing) / kNumericIncrement;
    }

    EXPECT_TRUE(J_n.isApprox(J_a, kNumericTolerance));
  }
}

TEST_F(CameraTests, DualityWithDistortion) {
  for (auto i = 0; i < kNumIterations; ++i) {
    const Landmark landmark = Eigen::internal::random<Scalar>(kMinLandmarkDistance, kMaxLandmarkDistance) * camera_.randomBearing();
    const auto bearing_0 = Camera::LandmarkToBearing(landmark);
    const auto pixel_0 = Camera::LandmarkToPixel(landmark);
    const auto normalized_pixel_0 = camera_.intrinsics().normalize(pixel_0);
    const auto distorted_pixel_0 = camera_.distortion() ? camera_.distortion()->distort(normalized_pixel_0, nullptr, nullptr, nullptr) : normalized_pixel_0;
    const auto normalized_pixel_1 = camera_.distortion() ? camera_.distortion()->undistort(distorted_pixel_0, nullptr, nullptr, nullptr) : distorted_pixel_0;
    const auto pixel_1 = camera_.intrinsics().denormalize(normalized_pixel_1);
    const auto bearing_1 = Camera::PixelToBearing(pixel_1);

    EXPECT_TRUE(bearing_1.isApprox(bearing_0, kNumericTolerance));
    EXPECT_TRUE(pixel_1.isApprox(pixel_0, kNumericTolerance));
    EXPECT_TRUE(normalized_pixel_1.isApprox(normalized_pixel_0, kNumericTolerance));
  }
}

}  // namespace hyper::sensors::tests
