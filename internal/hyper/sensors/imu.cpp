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

}  // namespace

IMU::IMU(std::unique_ptr<BiasInterpolator>&& gyroscope_bias_interpolator, std::unique_ptr<BiasInterpolator>&& accelerometer_bias_interpolator)
    : Sensor{Type::IMU, kNumVariables},
      gyroscope_noise_density_{},
      gyroscope_bias_{std::move(gyroscope_bias_interpolator)},
      accelerometer_noise_density_{},
      accelerometer_bias_{std::move(accelerometer_bias_interpolator)} {
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
}

IMU::IMU(const Node& node) : IMU{} {
  node >> *this;
}

auto IMU::variables() const -> Partitions<Variable*> {
  auto gyroscope_bias_variables = gyroscopeBias().variables();
  auto accelerometer_bias_variables = accelerometerBias().variables();
  return assembleVariables(gyroscope_bias_variables, accelerometer_bias_variables);
}

auto IMU::variables(const Time& time) const -> Partitions<Variable*> {
  auto gyroscope_bias_variables = gyroscopeBias().variables(time);
  auto accelerometer_bias_variables = accelerometerBias().variables(time);
  return assembleVariables(gyroscope_bias_variables, accelerometer_bias_variables);
}

auto IMU::parameterBlocks() const -> Partitions<Scalar*> {
  auto gyroscope_bias_parameter_blocks = gyroscopeBias().parameterBlocks();
  auto accelerometer_bias_parameter_blocks = accelerometerBias().parameterBlocks();
  return assembleParameterBlocks(gyroscope_bias_parameter_blocks, accelerometer_bias_parameter_blocks);
}

auto IMU::parameterBlocks(const Time& time) const -> Partitions<Scalar*> {
  auto gyroscope_bias_parameter_blocks = gyroscopeBias().parameterBlocks(time);
  auto accelerometer_bias_parameter_blocks = accelerometerBias().parameterBlocks(time);
  return assembleParameterBlocks(gyroscope_bias_parameter_blocks, accelerometer_bias_parameter_blocks);
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

auto IMU::assembleVariables(const std::vector<GyroscopeBias::StampedVariable*>& gyroscope_bias_variables,
                            const std::vector<AccelerometerBias::StampedVariable*>& accelerometer_bias_variables) const -> Partitions<Variable*> {
  const auto gyroscope_bias_offset = kVariablesOffset + variables_.size();
  const auto accelerometer_bias_offset = gyroscope_bias_offset + gyroscope_bias_variables.size();
  const auto num_variables = accelerometer_bias_offset + accelerometer_bias_variables.size();

  Partitions<Variable*> partitions;
  auto& [idxs, variables] = partitions;

  idxs.reserve(kNumPartitions);
  idxs.emplace_back(kVariablesOffset);
  idxs.emplace_back(gyroscope_bias_offset);
  idxs.emplace_back(accelerometer_bias_offset);

  variables.reserve(num_variables);
  std::transform(variables_.begin(), variables_.end(), std::back_inserter(variables), [](const auto& arg) { return arg.get(); });
  std::transform(gyroscope_bias_variables.begin(), gyroscope_bias_variables.end(), std::back_inserter(variables), [](const auto& arg) { return arg; });
  std::transform(accelerometer_bias_variables.begin(), accelerometer_bias_variables.end(), std::back_inserter(variables), [](const auto& arg) { return arg; });
  return partitions;
}

auto IMU::assembleParameterBlocks(const std::vector<Scalar*>& gyroscope_bias_parameter_blocks, const std::vector<Scalar*>& accelerometer_bias_parameter_blocks) const
    -> Partitions<Scalar*> {
  const auto gyroscope_bias_offset = kVariablesOffset + parameter_blocks_.size();
  const auto accelerometer_bias_offset = gyroscope_bias_offset + gyroscope_bias_parameter_blocks.size();
  const auto num_parameter_blocks = accelerometer_bias_offset + accelerometer_bias_parameter_blocks.size();

  Partitions<Scalar*> partitions;
  auto& [idxs, parameter_blocks] = partitions;

  idxs.reserve(kNumPartitions);
  idxs.emplace_back(kVariablesOffset);
  idxs.emplace_back(gyroscope_bias_offset);
  idxs.emplace_back(accelerometer_bias_offset);

  parameter_blocks.reserve(num_parameter_blocks);
  std::transform(parameter_blocks_.begin(), parameter_blocks_.end(), std::back_inserter(parameter_blocks), [](const auto& arg) { return arg; });
  std::transform(gyroscope_bias_parameter_blocks.begin(), gyroscope_bias_parameter_blocks.end(), std::back_inserter(parameter_blocks), [](const auto& arg) { return arg; });
  std::transform(accelerometer_bias_parameter_blocks.begin(), accelerometer_bias_parameter_blocks.end(), std::back_inserter(parameter_blocks), [](const auto& arg) { return arg; });
  return partitions;
}

}  // namespace hyper::sensors
