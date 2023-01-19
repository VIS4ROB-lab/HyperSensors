/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/sensors/imu.hpp"
#include "hyper/state/interpolators/basis.hpp"
#include "hyper/state/policies/cartesian.hpp"

namespace hyper::sensors {

namespace {

// Variable names.
constexpr auto kGyroscopeNoiseDensityName = "gyroscope_noise_density";
constexpr auto kGyroscopeIntrinsicsName = "gyroscope_intrinsics";
constexpr auto kGyroscopeSensitivityName = "gyroscope_sensitivity";
constexpr auto kAccelerometerNoiseDensityName = "accelerometer_noise_density";
constexpr auto kAccelerometerIntrinsicsName = "accelerometer_intrinsics";
constexpr auto kAccelerometerAxesOffsetsName = "accelerometer_axes_offsets";

/// Appends a vector and existing one.
/// \tparam TElement Element type.
/// \tparam TArgs Variadic argument types.
/// \param output Vector to append to.
/// \param args Vector to append.
template <typename TElement, typename... TArgs>
void appendVectors(std::vector<TElement>& output, const TArgs&... args) {
  (output.insert(output.end(), std::begin(args), std::end(args)), ...);
}

/// Concatenates vectors.
/// \tparam TArgs Variadic argument types.
/// \param args Input arguments.
/// \return Concatenated vectors.
template <typename... TArgs>
auto concatVectors(const TArgs&... args) {
  using Output = typename std::tuple_element<0, std::tuple<TArgs...>>::type;
  Output output;
  output.reserve((... + args.size()));
  appendVectors(output, args...);
  return output;
}

}  // namespace

IMU::IMU(const Node& node) : Sensor{kNumParameters, node}, gyroscope_noise_density_{}, gyroscope_bias_{}, accelerometer_noise_density_{}, accelerometer_bias_{} {
  initializeVariables();
  if (!node.IsNull()) {
    readVariables(node);
  }
}

auto IMU::parameters() const -> Pointers<Parameter> {
  return concatVectors(Sensor::parameters(), gyroscopeBias().parameters(), accelerometerBias().parameters());
}

auto IMU::parameters(const Stamp& stamp) const -> Pointers<Parameter> {
  return concatVectors(Sensor::parameters(stamp), gyroscopeBias().parameters(stamp), accelerometerBias().parameters(stamp));
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
  const auto vector = variableAsVector(kGyroscopeIntrinsicsOffset);
  return Eigen::Map<const GyroscopeIntrinsics>{vector.data()};
}

auto IMU::gyroscopeIntrinsics() -> Eigen::Map<GyroscopeIntrinsics> {
  auto vector = variableAsVector(kGyroscopeIntrinsicsOffset);
  return Eigen::Map<GyroscopeIntrinsics>{vector.data()};
}

auto IMU::gyroscopeSensitivity() const -> Eigen::Map<const GyroscopeSensitivity> {
  const auto vector = variableAsVector(kGyroscopeSensitivityOffset);
  return Eigen::Map<const GyroscopeSensitivity>{vector.data()};
}

auto IMU::gyroscopeSensitivity() -> Eigen::Map<GyroscopeSensitivity> {
  auto vector = variableAsVector(kGyroscopeSensitivityOffset);
  return Eigen::Map<GyroscopeSensitivity>{vector.data()};
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
  const auto vector = variableAsVector(kAccelerometerIntrinsicsOffset);
  return Eigen::Map<const AccelerometerIntrinsics>{vector.data()};
}

auto IMU::accelerometerIntrinsics() -> Eigen::Map<AccelerometerIntrinsics> {
  auto vector = variableAsVector(kAccelerometerIntrinsicsOffset);
  return Eigen::Map<AccelerometerIntrinsics>{vector.data()};
}

auto IMU::accelerometerAxesOffsets() const -> Eigen::Map<const AccelerometerAxesOffsets> {
  const auto vector = variableAsVector(kAccelerometerAxesOffsetsOffset);
  return Eigen::Map<const AccelerometerAxesOffsets>{vector.data()};
}

auto IMU::accelerometerAxesOffsets() -> Eigen::Map<AccelerometerAxesOffsets> {
  auto vector = variableAsVector(kAccelerometerAxesOffsetsOffset);
  return Eigen::Map<AccelerometerAxesOffsets>{vector.data()};
}

auto IMU::initializeVariables() -> void {
  DCHECK_LE(kNumParameters, variables_.size());
  variables_[kGyroscopeIntrinsicsOffset] = std::make_unique<GyroscopeIntrinsics>();
  variables_[kGyroscopeSensitivityOffset] = std::make_unique<GyroscopeSensitivity>();
  variables_[kAccelerometerIntrinsicsOffset] = std::make_unique<AccelerometerIntrinsics>();
  variables_[kAccelerometerAxesOffsetsOffset] = std::make_unique<AccelerometerAxesOffsets>();

  // Initialize gyroscope bias.
  auto gyroscope_bias_interpolator = std::make_unique<BasisInterpolator>(3, true);
  auto gyroscope_bias_policy = std::make_unique<CartesianPolicy<GyroscopeBias>>();
  gyroscope_bias_ = std::make_unique<AbstractState>(std::move(gyroscope_bias_interpolator), std::move(gyroscope_bias_policy));

  // Initialize accelerometer bias.
  auto accelerometer_bias_interpolator = std::make_unique<BasisInterpolator>(3, true);
  auto accelerometer_bias_policy = std::make_unique<CartesianPolicy<AccelerometerBias>>();
  accelerometer_bias_ = std::make_unique<AbstractState>(std::move(accelerometer_bias_interpolator), std::move(accelerometer_bias_policy));
}

auto IMU::readVariables(const Node& node) -> void {
  gyroscopeIntrinsics() = yaml::ReadVariable<GyroscopeIntrinsics>(node, kGyroscopeIntrinsicsName);
  gyroscopeSensitivity() = yaml::ReadVariable<GyroscopeSensitivity>(node, kGyroscopeSensitivityName);
  gyroscopeNoiseDensity() = yaml::ReadAs<GyroscopeNoiseDensity>(node, kGyroscopeNoiseDensityName);
  accelerometerIntrinsics() = yaml::ReadVariable<AccelerometerIntrinsics>(node, kAccelerometerIntrinsicsName);
  accelerometerAxesOffsets() = yaml::ReadVariable<AccelerometerAxesOffsets>(node, kAccelerometerAxesOffsetsName);
  accelerometerNoiseDensity() = yaml::ReadAs<AccelerometerNoiseDensity>(node, kAccelerometerNoiseDensityName);
}

auto IMU::writeVariables(Emitter& emitter) const -> void {
  Sensor::writeVariables(emitter);
  yaml::WriteVariable(emitter, kGyroscopeIntrinsicsName, gyroscopeIntrinsics());
  yaml::WriteVariable(emitter, kGyroscopeSensitivityName, gyroscopeSensitivity());
  yaml::Write(emitter, kGyroscopeNoiseDensityName, gyroscopeNoiseDensity());
  yaml::WriteVariable(emitter, kAccelerometerIntrinsicsName, accelerometerIntrinsics());
  yaml::WriteVariable(emitter, kAccelerometerAxesOffsetsName, accelerometerAxesOffsets());
  yaml::Write(emitter, kAccelerometerNoiseDensityName, accelerometerNoiseDensity());
}

}  // namespace hyper::sensors
