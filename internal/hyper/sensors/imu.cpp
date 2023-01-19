/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/sensors/imu.hpp"
#include "hyper/state/interpolators/spatial/spatial.hpp"
#include "hyper/state/interpolators/temporal/basis.hpp"

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

IMU::IMU(const Node& node) : Sensor{kNumVariables, node}, gyroscope_noise_density_{}, gyroscope_bias_{}, accelerometer_noise_density_{}, accelerometer_bias_{} {
  // Initialize variables.
  DCHECK_LE(kNumVariables, variables_.size());
  variables_[kGyroscopeIntrinsicsOffset] = std::make_unique<GyroscopeIntrinsics>();
  variables_[kGyroscopeSensitivityOffset] = std::make_unique<GyroscopeSensitivity>();
  variables_[kAccelerometerIntrinsicsOffset] = std::make_unique<AccelerometerIntrinsics>();
  variables_[kAccelerometerAxesOffsetsOffset] = std::make_unique<AccelerometerAxesOffsets>();
  parameters_[kGyroscopeIntrinsicsOffset] = variables_[kGyroscopeIntrinsicsOffset]->asVector().data();
  parameters_[kGyroscopeSensitivityOffset] = variables_[kGyroscopeSensitivityOffset]->asVector().data();
  parameters_[kAccelerometerIntrinsicsOffset] = variables_[kAccelerometerIntrinsicsOffset]->asVector().data();
  parameters_[kAccelerometerAxesOffsetsOffset] = variables_[kAccelerometerAxesOffsetsOffset]->asVector().data();

  // Initialize gyroscope and accelerometer bias.
  static auto interpolator = state::BasisInterpolator<Scalar, 4>{};
  gyroscope_bias_.setInterpolator(&interpolator);
  accelerometer_bias_.setInterpolator(&interpolator);

  if (!node.IsNull()) {
    read(node);
  }
}

auto IMU::pointers() const -> std::vector<Variable*> {
  return concatVectors(Sensor::pointers(), gyroscopeBias().pointers(), accelerometerBias().pointers());
}

auto IMU::pointers(const Time& time) const -> std::vector<Variable*> {
  return concatVectors(Sensor::pointers(time), gyroscopeBias().pointers(time), accelerometerBias().pointers(time));
}

auto IMU::parameters() const -> std::vector<Scalar*> {
  return concatVectors(Sensor::parameters(), gyroscopeBias().parameters(), accelerometerBias().parameters());
}

auto IMU::parameters(const Time& time) const -> std::vector<Scalar*> {
  return concatVectors(Sensor::parameters(time), gyroscopeBias().parameters(time), accelerometerBias().parameters(time));
}

auto IMU::gyroscopeNoiseDensity() const -> const GyroscopeNoiseDensity& {
  return gyroscope_noise_density_;
}

auto IMU::gyroscopeNoiseDensity() -> GyroscopeNoiseDensity& {
  return const_cast<GyroscopeNoiseDensity&>(std::as_const(*this).gyroscopeNoiseDensity());
}

auto IMU::gyroscopeBias() const -> const GyroscopeBias& {
  return gyroscope_bias_;
}

auto IMU::gyroscopeBias() -> GyroscopeBias& {
  return const_cast<GyroscopeBias&>(std::as_const(*this).gyroscopeBias());
}

auto IMU::gyroscopeIntrinsics() const -> Eigen::Map<const GyroscopeIntrinsics> {
  return Eigen::Map<const GyroscopeIntrinsics>{parameters_[kGyroscopeIntrinsicsOffset]};
}

auto IMU::gyroscopeIntrinsics() -> Eigen::Map<GyroscopeIntrinsics> {
  return Eigen::Map<GyroscopeIntrinsics>{parameters_[kGyroscopeIntrinsicsOffset]};
}

auto IMU::gyroscopeSensitivity() const -> Eigen::Map<const GyroscopeSensitivity> {
  return Eigen::Map<const GyroscopeSensitivity>{parameters_[kGyroscopeSensitivityOffset]};
}

auto IMU::gyroscopeSensitivity() -> Eigen::Map<GyroscopeSensitivity> {
  return Eigen::Map<GyroscopeSensitivity>{parameters_[kGyroscopeSensitivityOffset]};
}

auto IMU::accelerometerNoiseDensity() const -> const AccelerometerNoiseDensity& {
  return accelerometer_noise_density_;
}

auto IMU::accelerometerNoiseDensity() -> AccelerometerNoiseDensity& {
  return const_cast<AccelerometerNoiseDensity&>(std::as_const(*this).accelerometerNoiseDensity());
}

auto IMU::accelerometerBias() const -> const AccelerometerBias& {
  return accelerometer_bias_;
}

auto IMU::accelerometerBias() -> AccelerometerBias& {
  return const_cast<AccelerometerBias&>(std::as_const(*this).accelerometerBias());
}

auto IMU::accelerometerIntrinsics() const -> Eigen::Map<const AccelerometerIntrinsics> {
  return Eigen::Map<const AccelerometerIntrinsics>{parameters_[kAccelerometerIntrinsicsOffset]};
}

auto IMU::accelerometerIntrinsics() -> Eigen::Map<AccelerometerIntrinsics> {
  return Eigen::Map<AccelerometerIntrinsics>{parameters_[kAccelerometerIntrinsicsOffset]};
}

auto IMU::accelerometerAxesOffsets() const -> Eigen::Map<const AccelerometerAxesOffsets> {
  return Eigen::Map<const AccelerometerAxesOffsets>{parameters_[kAccelerometerAxesOffsetsOffset]};
}

auto IMU::accelerometerAxesOffsets() -> Eigen::Map<AccelerometerAxesOffsets> {
  return Eigen::Map<AccelerometerAxesOffsets>{parameters_[kAccelerometerAxesOffsetsOffset]};
}

auto IMU::read(const Node& node) -> void {
  gyroscopeIntrinsics() = yaml::ReadVariable<GyroscopeIntrinsics>(node, kGyroscopeIntrinsicsName);
  gyroscopeSensitivity() = yaml::ReadVariable<GyroscopeSensitivity>(node, kGyroscopeSensitivityName);
  gyroscopeNoiseDensity() = yaml::ReadAs<GyroscopeNoiseDensity>(node, kGyroscopeNoiseDensityName);
  accelerometerIntrinsics() = yaml::ReadVariable<AccelerometerIntrinsics>(node, kAccelerometerIntrinsicsName);
  accelerometerAxesOffsets() = yaml::ReadVariable<AccelerometerAxesOffsets>(node, kAccelerometerAxesOffsetsName);
  accelerometerNoiseDensity() = yaml::ReadAs<AccelerometerNoiseDensity>(node, kAccelerometerNoiseDensityName);
}

auto IMU::write(Emitter& emitter) const -> void {
  Sensor::write(emitter);
  yaml::WriteVariable(emitter, kGyroscopeIntrinsicsName, gyroscopeIntrinsics());
  yaml::WriteVariable(emitter, kGyroscopeSensitivityName, gyroscopeSensitivity());
  yaml::Write(emitter, kGyroscopeNoiseDensityName, gyroscopeNoiseDensity());
  yaml::WriteVariable(emitter, kAccelerometerIntrinsicsName, accelerometerIntrinsics());
  yaml::WriteVariable(emitter, kAccelerometerAxesOffsetsName, accelerometerAxesOffsets());
  yaml::Write(emitter, kAccelerometerNoiseDensityName, accelerometerNoiseDensity());
}

}  // namespace hyper::sensors
