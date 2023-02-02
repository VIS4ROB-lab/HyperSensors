/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/sensors/imu.hpp"
#include "hyper/state/interpolators/interpolators.hpp"

namespace hyper::sensors {

namespace {

// Variable names.
constexpr auto kGyroscopeNoiseDensityName = "gyroscope_noise_density";
constexpr auto kGyroscopeIntrinsicsName = "gyroscope_intrinsics";
constexpr auto kGyroscopeSensitivityName = "gyroscope_sensitivity";
constexpr auto kAccelerometerNoiseDensityName = "accelerometer_noise_density";
constexpr auto kAccelerometerIntrinsicsName = "accelerometer_intrinsics";
constexpr auto kAccelerometerOffsetName = "accelerometer_offset";

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

IMU::IMU() : Sensor{kNumVariables}, gyroscope_noise_density_{}, gyroscope_bias_{}, accelerometer_noise_density_{}, accelerometer_bias_{} {
  // Initialize variables.
  DCHECK_LE(kNumVariables, variables_.size());
  variables_[kGyroscopeIntrinsicsIndex] = std::make_unique<GyroscopeIntrinsics>();
  variables_[kGyroscopeSensitivityIndex] = std::make_unique<GyroscopeSensitivity>();
  variables_[kAccelerometerIntrinsicsIndex] = std::make_unique<AccelerometerIntrinsics>();
  variables_[kAccelerometerOffsetIndex] = std::make_unique<AccelerometerOffset>();
  parameter_blocks_[kGyroscopeIntrinsicsIndex] = variables_[kGyroscopeIntrinsicsIndex]->asVector().data();
  parameter_blocks_[kGyroscopeSensitivityIndex] = variables_[kGyroscopeSensitivityIndex]->asVector().data();
  parameter_blocks_[kAccelerometerIntrinsicsIndex] = variables_[kAccelerometerIntrinsicsIndex]->asVector().data();
  parameter_blocks_[kAccelerometerOffsetIndex] = variables_[kAccelerometerOffsetIndex]->asVector().data();

  // Initialize gyroscope and accelerometer bias.
  static auto interpolator = state::BasisInterpolator<Scalar, 4>{};
  gyroscope_bias_.setInterpolator(&interpolator);
  accelerometer_bias_.setInterpolator(&interpolator);
}

auto IMU::variables() const -> std::vector<Variable*> {
  return concatVectors(Sensor::variables(), gyroscopeBias().inputs(), accelerometerBias().inputs());
}

auto IMU::variables(const Time& time) const -> std::vector<Variable*> {
  return concatVectors(Sensor::variables(time), gyroscopeBias().inputs(time), accelerometerBias().inputs(time));
}

auto IMU::parameterBlocks() const -> std::vector<Scalar*> {
  return concatVectors(Sensor::parameterBlocks(), gyroscopeBias().parameterBlocks(), accelerometerBias().parameterBlocks());
}

auto IMU::parameterBlocks(const Time& time) const -> std::vector<Scalar*> {
  return concatVectors(Sensor::parameterBlocks(time), gyroscopeBias().parameterBlocks(time), accelerometerBias().parameterBlocks(time));
}

auto IMU::gyroscopeNoiseDensity() const -> const GyroscopeNoiseDensity& {
  return gyroscope_noise_density_;
}

auto IMU::gyroscopeNoiseDensity() -> GyroscopeNoiseDensity& {
  return const_cast<GyroscopeNoiseDensity&>(std::as_const(*this).gyroscopeNoiseDensity());
}

auto IMU::gyroscopeIntrinsics() const -> const GyroscopeIntrinsics& {
  return static_cast<const GyroscopeIntrinsics&>(*variables_[kGyroscopeIntrinsicsIndex]);  // NOLINT
}

auto IMU::gyroscopeIntrinsics() -> GyroscopeIntrinsics& {
  return const_cast<GyroscopeIntrinsics&>(std::as_const(*this).gyroscopeIntrinsics());
}

auto IMU::gyroscopeSensitivity() const -> const GyroscopeSensitivity& {
  return static_cast<const GyroscopeSensitivity&>(*variables_[kGyroscopeSensitivityIndex]);  // NOLINT
}

auto IMU::gyroscopeSensitivity() -> GyroscopeSensitivity& {
  return const_cast<GyroscopeSensitivity&>(std::as_const(*this).gyroscopeSensitivity());
}

auto IMU::gyroscopeBias() const -> const GyroscopeBias& {
  return gyroscope_bias_;
}

auto IMU::gyroscopeBias() -> GyroscopeBias& {
  return const_cast<GyroscopeBias&>(std::as_const(*this).gyroscopeBias());
}

auto IMU::accelerometerNoiseDensity() const -> const AccelerometerNoiseDensity& {
  return accelerometer_noise_density_;
}

auto IMU::accelerometerNoiseDensity() -> AccelerometerNoiseDensity& {
  return const_cast<AccelerometerNoiseDensity&>(std::as_const(*this).accelerometerNoiseDensity());
}

auto IMU::accelerometerIntrinsics() const -> const AccelerometerIntrinsics& {
  return static_cast<const AccelerometerIntrinsics&>(*variables_[kAccelerometerIntrinsicsIndex]);  // NOLINT
}

auto IMU::accelerometerIntrinsics() -> AccelerometerIntrinsics& {
  return const_cast<AccelerometerIntrinsics&>(std::as_const(*this).accelerometerIntrinsics());
}

auto IMU::accelerometerOffset() const -> const AccelerometerOffset& {
  return static_cast<AccelerometerOffset&>(*variables_[kAccelerometerOffsetIndex]);  //NOLINT
}

auto IMU::accelerometerOffset() -> AccelerometerOffset& {
  return const_cast<AccelerometerOffset&>(std::as_const(*this).accelerometerOffset());
}

auto IMU::accelerometerBias() const -> const AccelerometerBias& {
  return accelerometer_bias_;
}

auto IMU::accelerometerBias() -> AccelerometerBias& {
  return const_cast<AccelerometerBias&>(std::as_const(*this).accelerometerBias());
}

auto IMU::read(const Node& node) -> void {
  Sensor::read(node);
  gyroscopeNoiseDensity() = yaml::ReadAs<GyroscopeNoiseDensity>(node, kGyroscopeNoiseDensityName);
  gyroscopeIntrinsics() = yaml::ReadVariable<GyroscopeIntrinsics>(node, kGyroscopeIntrinsicsName);
  gyroscopeSensitivity() = yaml::ReadVariable<GyroscopeSensitivity>(node, kGyroscopeSensitivityName);
  accelerometerNoiseDensity() = yaml::ReadAs<AccelerometerNoiseDensity>(node, kAccelerometerNoiseDensityName);
  accelerometerIntrinsics() = yaml::ReadVariable<AccelerometerIntrinsics>(node, kAccelerometerIntrinsicsName);
  accelerometerOffset() = yaml::ReadVariable<AccelerometerOffset>(node, kAccelerometerOffsetName);
}

auto IMU::write(Emitter& emitter) const -> void {
  Sensor::write(emitter);
  yaml::Write(emitter, kGyroscopeNoiseDensityName, gyroscopeNoiseDensity());
  yaml::WriteVariable(emitter, kGyroscopeIntrinsicsName, gyroscopeIntrinsics());
  yaml::WriteVariable(emitter, kGyroscopeSensitivityName, gyroscopeSensitivity());
  yaml::Write(emitter, kAccelerometerNoiseDensityName, accelerometerNoiseDensity());
  yaml::WriteVariable(emitter, kAccelerometerIntrinsicsName, accelerometerIntrinsics());
  yaml::WriteVariable(emitter, kAccelerometerOffsetName, accelerometerOffset());
}

}  // namespace hyper::sensors
