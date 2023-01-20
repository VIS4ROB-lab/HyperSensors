/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "hyper/sensors/camera.hpp"

namespace hyper::test {

class CameraTests : public testing::Test {
 protected:
  // Constants.
  static constexpr auto kFilePath = __FILE__;
  static constexpr auto kInputFileName = "camera.yaml";
  static constexpr auto kOutputFileName = "camera_out.yaml";

  // Definitions.
  using Path = std::filesystem::path;
  using Camera = sensors::Camera;

  /// Setup.
  auto SetUp() -> void final {
    input_ = Path{kFilePath}.parent_path() /= kInputFileName;
    YAML::LoadFile(input_) >> camera_;
  }

  /// Compares to files.
  /// \param p1 Path to first file.
  /// \param p2 Path to second file.
  /// \return True if files are identical.
  static auto CompareFiles(const Path& p1, const Path& p2) -> bool {
    std::ifstream f1(p1, std::ifstream::binary | std::ifstream::ate);
    std::ifstream f2(p2, std::ifstream::binary | std::ifstream::ate);

    if (f1.fail() || f2.fail()) {
      return false;
    }

    if (f1.tellg() != f2.tellg()) {
      return false;
    }

    f1.seekg(0, std::ifstream::beg);
    f2.seekg(0, std::ifstream::beg);
    return std::equal(std::istreambuf_iterator<char>(f1.rdbuf()), std::istreambuf_iterator<char>(), std::istreambuf_iterator<char>(f2.rdbuf()));
  }

  Path input_;
  Camera camera_;
};

TEST_F(CameraTests, ReadWrite) {
  YAML::Emitter emitter;
  const auto output = Path{kFilePath}.parent_path() /= kOutputFileName;
  std::ofstream{output} << (emitter << camera_).c_str();
  EXPECT_TRUE(CompareFiles(input_, output));
  std::remove(output.c_str());
}

/* auto TestHelper<Camera>::RandomPosition() -> Position {
  Position position = Position::Random();
  if (position.z() < Scalar{0})
    position.z() = -position.z();
  return position;
}

auto TestHelper<Camera>::Mock() -> std::unique_ptr<Camera> {
  // Set the magnitude of perturbations.
  constexpr auto kMaxPerturbation = 0.05;

  // Create a distortion.
  using Distortion = RadialTangentialDistortion<2>;
  auto distortion = std::make_unique<Distortion>(-0.28340811, 0.07395907, 1.76187114e-05, 0.00019359);
  distortion->cartesian() += kMaxPerturbation * Distortion::Cartesian::Random();

  // Create the camera.
  auto camera = std::make_unique<Camera>();
  camera->se3() = perturb(camera->se3(), 0.05);
  camera->sensorSize() = {752, 480};
  camera->intrinsics() = Traits<Camera>::Intrinsics{367.215, 248.375, 458.654, 457.296};
  camera->setDistortion(std::move(distortion));
  return camera;
}

auto TestHelper<Camera>::ExpectEquality(const Camera& sensor_0, const Camera& sensor_1) -> void {
  using Traits = TestTraits<Scalar>;
  TestHelper<Sensor>::ExpectEquality(sensor_0, sensor_1);
  EXPECT_TRUE(sensor_0.sensorSize().width == sensor_1.sensorSize().width);
  EXPECT_TRUE(sensor_0.sensorSize().height == sensor_1.sensorSize().height);
  EXPECT_TRUE(sensor_0.intrinsics().isApprox(sensor_1.intrinsics(), Traits::kNumericTolerance));
  EXPECT_TRUE(isApprox(sensor_0.distortion(), sensor_1.distortion(), Traits::kNumericTolerance));
}

/// Tests the read and write consistency.
TEST(CameraTests, WriteAndRead) {  // NOLINT
  const auto out_sensor = TestHelper<Camera>::Mock();
  const auto in_sensor = TestHelper<Sensor>::WriteAndRead(*out_sensor);
  TestHelper<Camera>::ExpectEquality(*out_sensor, *in_sensor);
}

/// Tests the consistency of projection and lift operations.
TEST(CameraTests, ProjectAndLiftDuality) {  // NOLINT
  using Traits = TestTraits<Scalar>;
  constexpr auto kNumIterations = 25;
  for (auto i = 0; i < kNumIterations; ++i) {
    const Position position = TestHelper<Camera>::RandomPosition();
    const Pixel pixel = Camera::ProjectToPlane(position);
    const Bearing bearing_0 = Camera::ProjectToSphere(position);
    const Bearing bearing_1 = Camera::LiftToSphere(pixel);
    EXPECT_TRUE(Traits::ApproximateNormalizedEquality(bearing_1, bearing_0));
  }
}

/// Tests the Jacobian for the projection to the image plane.
TEST(CameraTests, ProjectToPlaneJacobian) {  // NOLINT
  using Traits = TestTraits<Scalar>;
  constexpr auto kNumIterations = 25;
  for (auto i = 0; i < kNumIterations; ++i) {
    const Position position = TestHelper<Camera>::RandomPosition();

    Jacobian<Pixel, Position> analytic_jacobian;
    const Pixel pixel = Camera::ProjectToPlane(position, analytic_jacobian.data());

    Jacobian<Pixel, Position> numeric_jacobian;
    for (auto j = 0; j < Position::SizeAtCompileTime; ++j) {
      const Position d_position = position + Traits::kNumericIncrement * Position::Unit(j);
      const Pixel d_pixel = Camera::ProjectToPlane(d_position);
      numeric_jacobian.col(j) = (d_pixel - pixel) / Traits::kNumericIncrement;
    }

    EXPECT_TRUE(Traits::ApproximateNormalizedEquality(numeric_jacobian, analytic_jacobian));
  }
}

/// Tests the Jacobian for the projection to the unit sphere.
TEST(CameraTests, ProjectToSphereJacobian) {  // NOLINT
  using Traits = TestTraits<Scalar>;
  constexpr auto kNumIterations = 25;
  for (auto i = 0; i < kNumIterations; ++i) {
    const Position position = TestHelper<Camera>::RandomPosition();

    Jacobian<Bearing, Position> analytic_jacobian;
    const Bearing bearing = Camera::ProjectToSphere(position, analytic_jacobian.data());

    Jacobian<Bearing, Position> numeric_jacobian;
    for (auto j = 0; j < Position::SizeAtCompileTime; ++j) {
      const Position d_position = position + Traits::kNumericIncrement * Position::Unit(j);
      const Bearing d_bearing = Camera::ProjectToSphere(d_position);
      numeric_jacobian.col(j) = (d_bearing - bearing) / Traits::kNumericIncrement;
    }

    EXPECT_TRUE(Traits::ApproximateNormalizedEquality(numeric_jacobian, analytic_jacobian));
  }
}

/// Tests the Jacobian for lifting a pixel to the unit sphere.
TEST(CameraTests, LiftToSphereJacobian) {  // NOLINT
  using Traits = TestTraits<Scalar>;
  constexpr auto kNumIterations = 25;
  for (auto i = 0; i < kNumIterations; ++i) {
    const Pixel pixel = Pixel::Random();

    Jacobian<Bearing, Pixel> analytic_jacobian;
    const Bearing bearing = Camera::LiftToSphere(pixel, analytic_jacobian.data());

    Jacobian<Bearing, Pixel> numeric_jacobian;
    for (auto j = 0; j < Pixel::SizeAtCompileTime; ++j) {
      const Pixel d_pixel = pixel + Traits::kNumericIncrement * Pixel::Unit(j);
      const Bearing d_bearing = Camera::LiftToSphere(d_pixel);
      numeric_jacobian.col(j) = (d_bearing - bearing) / Traits::kNumericIncrement;
    }

    EXPECT_TRUE(Traits::ApproximateNormalizedEquality(numeric_jacobian, analytic_jacobian));
  }
}

/// Tests the full duality of all combined operators.
TEST(CameraTests, FullDuality) {  // NOLINT
  using Traits = TestTraits<Scalar>;
  constexpr auto kNumOuterIterations = 5;
  constexpr auto kNumInnerIterations = 25;
  for (auto i = 0; i < kNumOuterIterations; ++i) {
    // Create mock camera.
    const auto sensor = TestHelper<Camera>::Mock();

    // Project and lift random positions.
    for (auto j = 0; j < kNumInnerIterations; ++j) {
      const Position position = TestHelper<Camera>::RandomPosition();
      const Bearing bearing_0 = Camera::ProjectToSphere(position);
      const Pixel pixel_0 = Camera::ProjectToPlane(position);
      const Pixel normalized_pixel_0 = sensor->intrinsics().normalizePixel(pixel_0);
      const Pixel distorted_pixel_0 = sensor->distortion().distort(normalized_pixel_0.data(), nullptr, nullptr);
      const Pixel normalized_pixel_1 = sensor->distortion().undistort(distorted_pixel_0);
      const Pixel pixel_1 = sensor->intrinsics().denormalizePixel(normalized_pixel_1);
      const Bearing bearing_1 = Camera::LiftToSphere(pixel_1);

      EXPECT_TRUE(Traits::ApproximateNormalizedEquality(bearing_1, bearing_0));
      EXPECT_TRUE(Traits::ApproximateNormalizedEquality(pixel_1, pixel_0));
      EXPECT_TRUE(Traits::ApproximateNormalizedEquality(normalized_pixel_1, normalized_pixel_0));
    }
  }
} */

}  // namespace hyper::test
