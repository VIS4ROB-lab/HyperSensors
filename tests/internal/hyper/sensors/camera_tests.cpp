/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/sensors/camera.hpp"
#include "hyper/variables/distortions/distortions.hpp"

/* namespace hyper::test {

auto TestHelper<Camera>::RandomPosition() -> Position {
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
}

}  // namespace hyper::test */