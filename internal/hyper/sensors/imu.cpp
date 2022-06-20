/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/sensors/imu.hpp"
#include "hyper/state/interpolators/basis.hpp"
#include "hyper/state/policies/cartesian.hpp"

namespace hyper {

namespace {

// Parameter names.
constexpr auto kGyroscopeNoiseDensityName = "gyroscope_noise_density";
constexpr auto kGyroscopeIntrinsicsName = "gyroscope_intrinsics";
constexpr auto kGyroscopeSensitivityName = "gyroscope_sensitivity";
constexpr auto kAccelerometerNoiseDensityName = "accelerometer_noise_density";
constexpr auto kAccelerometerIntrinsicsName = "accelerometer_intrinsics";
constexpr auto kAccelerometerAxesOffsetsName = "accelerometer_axes_offsets";

template <typename TElement, typename... TArgs>
void appendVectors(std::vector<TElement>& output, const TArgs&... args) {
  (output.insert(output.end(), std::begin(args), std::end(args)), ...);
}

template <typename... TArgs>
auto concatVectors(const TArgs&... args) {
  using Container = typename std::tuple_element<0, std::tuple<TArgs...>>::type;
  Container output;
  output.reserve((... + args.size()));
  appendVectors(output, args...);
  return output;
}

} // namespace

IMU::IMU(const Node& node)
    : Sensor{Traits<IMU>::kNumParameters, node},
      gyroscope_noise_density_{},
      gyroscope_bias_{},
      accelerometer_noise_density_{},
      accelerometer_bias_{} {
  initializeParameters();
  if (!node.IsNull()) {
    readParameters(node);
  }
}

auto IMU::memoryBlocks() const -> MemoryBlocks<Scalar> {
  return concatVectors(Sensor::memoryBlocks(), gyroscopeBias().memoryBlocks(), accelerometerBias().memoryBlocks());
}

auto IMU::memoryBlocks(const Stamp& stamp) const -> MemoryBlocks<Scalar> {
  return concatVectors(Sensor::memoryBlocks(stamp), gyroscopeBias().memoryBlocks(stamp), accelerometerBias().memoryBlocks(stamp));
}

auto IMU::gyroscopeNoiseDensity() const -> const GyroscopeNoiseDensity& {
  return gyroscope_noise_density_;
}

auto IMU::gyroscopeNoiseDensity() -> GyroscopeNoiseDensity& {
  return const_cast<GyroscopeNoiseDensity&>(std::as_const(*this).gyroscopeNoiseDensity());
}

auto IMU::gyroscopeBias() const -> const AbstractState& {
  DCHECK(gyroscope_bias_ != nullptr);
  return *gyroscope_bias_;
}

auto IMU::gyroscopeBias() -> AbstractState& {
  return const_cast<AbstractState&>(std::as_const(*this).gyroscopeBias());
}

auto IMU::gyroscopeIntrinsics() const -> Eigen::Map<const GyroscopeIntrinsics> {
  return Eigen::Map<const GyroscopeIntrinsics>{address(Traits<IMU>::kGyroscopeIntrinsicsOffset)};
}

auto IMU::gyroscopeIntrinsics() -> Eigen::Map<GyroscopeIntrinsics> {
  return Eigen::Map<GyroscopeIntrinsics>{address(Traits<IMU>::kGyroscopeIntrinsicsOffset)};
}

auto IMU::gyroscopeSensitivity() const -> Eigen::Map<const GyroscopeSensitivity> {
  return Eigen::Map<const GyroscopeSensitivity>{address(Traits<IMU>::kGyroscopeSensitivityOffset)};
}

auto IMU::gyroscopeSensitivity() -> Eigen::Map<GyroscopeSensitivity> {
  return Eigen::Map<GyroscopeSensitivity>{address(Traits<IMU>::kGyroscopeSensitivityOffset)};
}

auto IMU::accelerometerNoiseDensity() const -> const AccelerometerNoiseDensity& {
  return accelerometer_noise_density_;
}

auto IMU::accelerometerNoiseDensity() -> AccelerometerNoiseDensity& {
  return const_cast<AccelerometerNoiseDensity&>(std::as_const(*this).accelerometerNoiseDensity());
}

auto IMU::accelerometerBias() const -> const AbstractState& {
  DCHECK(accelerometer_bias_ != nullptr);
  return *accelerometer_bias_;
}

auto IMU::accelerometerBias() -> AbstractState& {
  return const_cast<AbstractState&>(std::as_const(*this).accelerometerBias());
}

auto IMU::accelerometerIntrinsics() const -> Eigen::Map<const AccelerometerIntrinsics> {
  return Eigen::Map<const AccelerometerIntrinsics>{address(Traits<IMU>::kAccelerometerIntrinsicsOffset)};
}

auto IMU::accelerometerIntrinsics() -> Eigen::Map<AccelerometerIntrinsics> {
  return Eigen::Map<AccelerometerIntrinsics>{address(Traits<IMU>::kAccelerometerIntrinsicsOffset)};
}

auto IMU::accelerometerAxesOffsets() const -> Eigen::Map<const AccelerometerAxesOffsets> {
  return Eigen::Map<const AccelerometerAxesOffsets>{address(Traits<IMU>::kAccelerometerAxesOffsetsOffset)};
}

auto IMU::accelerometerAxesOffsets() -> Eigen::Map<AccelerometerAxesOffsets> {
  return Eigen::Map<AccelerometerAxesOffsets>{address(Traits<IMU>::kAccelerometerAxesOffsetsOffset)};
}

auto IMU::initializeParameters() -> void {
  DCHECK_LE(Traits<IMU>::kNumParameters, parameters().variables().size());
  parameters_.setVariable(Traits<IMU>::kGyroscopeIntrinsicsOffset, std::make_unique<GyroscopeIntrinsics>());
  parameters_.setVariable(Traits<IMU>::kGyroscopeSensitivityOffset, std::make_unique<GyroscopeSensitivity>());
  parameters_.setVariable(Traits<IMU>::kAccelerometerIntrinsicsOffset, std::make_unique<AccelerometerIntrinsics>());
  parameters_.setVariable(Traits<IMU>::kAccelerometerAxesOffsetsOffset, std::make_unique<AccelerometerAxesOffsets>());

  // Initialize gyroscope bias.
  auto gyroscope_bias_interpolator = std::make_unique<BasisInterpolator>(3, true);
  auto gyroscope_bias_policy = std::make_unique<CartesianPolicy<GyroscopeBias>>();
  gyroscope_bias_ = std::make_unique<AbstractState>(std::move(gyroscope_bias_interpolator), std::move(gyroscope_bias_policy));

  // Initialize accelerometer bias.
  auto accelerometer_bias_interpolator = std::make_unique<BasisInterpolator>(3, true);
  auto accelerometer_bias_policy = std::make_unique<CartesianPolicy<AccelerometerBias>>();
  accelerometer_bias_ = std::make_unique<AbstractState>(std::move(accelerometer_bias_interpolator), std::move(accelerometer_bias_policy));
}

auto IMU::readParameters(const Node& node) -> void {
  gyroscopeIntrinsics() = yaml::ReadVariable<GyroscopeIntrinsics>(node, kGyroscopeIntrinsicsName);
  gyroscopeSensitivity() = yaml::ReadVariable<GyroscopeSensitivity>(node, kGyroscopeSensitivityName);
  gyroscopeNoiseDensity() = yaml::ReadAs<GyroscopeNoiseDensity>(node, kGyroscopeNoiseDensityName);
  accelerometerIntrinsics() = yaml::ReadVariable<AccelerometerIntrinsics>(node, kAccelerometerIntrinsicsName);
  accelerometerAxesOffsets() = yaml::ReadVariable<AccelerometerAxesOffsets>(node, kAccelerometerAxesOffsetsName);
  accelerometerNoiseDensity() = yaml::ReadAs<AccelerometerNoiseDensity>(node, kAccelerometerNoiseDensityName);
}

auto IMU::writeParameters(Emitter& emitter) const -> void {
  Sensor::writeParameters(emitter);
  yaml::WriteVariable(emitter, kGyroscopeIntrinsicsName, gyroscopeIntrinsics());
  yaml::WriteVariable(emitter, kGyroscopeSensitivityName, gyroscopeSensitivity());
  yaml::Write(emitter, kGyroscopeNoiseDensityName, gyroscopeNoiseDensity());
  yaml::WriteVariable(emitter, kAccelerometerIntrinsicsName, accelerometerIntrinsics());
  yaml::WriteVariable(emitter, kAccelerometerAxesOffsetsName, accelerometerAxesOffsets());
  yaml::Write(emitter, kAccelerometerNoiseDensityName, accelerometerNoiseDensity());
}

} // namespace hyper
