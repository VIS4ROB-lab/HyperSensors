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

IMU::IMU(JacobianType jacobian_type, std::unique_ptr<BiasInterpolator>&& gyroscope_bias_interpolator, std::unique_ptr<BiasInterpolator>&& accelerometer_bias_interpolator)
    : Sensor{Type::IMU, jacobian_type, kNumVariables},
      gyroscope_noise_density_{},
      gyroscope_bias_{std::move(gyroscope_bias_interpolator)},
      accelerometer_noise_density_{},
      accelerometer_bias_{std::move(accelerometer_bias_interpolator)} {
  // Initialize variables.
  DCHECK_EQ(kNumVariables, variables_.size());
  DCHECK_EQ(kNumVariables, parameter_blocks_.size());
  variables_[kGyroscopeIntrinsicsIndex] = std::make_unique<GyroscopeIntrinsics>();
  variables_[kGyroscopeSensitivityIndex] = std::make_unique<GyroscopeSensitivity>();
  variables_[kAccelerometerIntrinsicsIndex] = std::make_unique<AccelerometerIntrinsics>();
  variables_[kAccelerometerOffsetIndex] = std::make_unique<AccelerometerOffset>();
  parameter_blocks_[kGyroscopeIntrinsicsIndex] = variables_[kGyroscopeIntrinsicsIndex]->asVector().data();
  parameter_blocks_[kGyroscopeSensitivityIndex] = variables_[kGyroscopeSensitivityIndex]->asVector().data();
  parameter_blocks_[kAccelerometerIntrinsicsIndex] = variables_[kAccelerometerIntrinsicsIndex]->asVector().data();
  parameter_blocks_[kAccelerometerOffsetIndex] = variables_[kAccelerometerOffsetIndex]->asVector().data();
  updateIMUParameterBlockSizes();
}

IMU::IMU(const Node& node) : IMU{} {
  node >> *this;
}

auto IMU::partitions(const Time& time) const -> Partitions<Scalar*> {
  return assemblePartitions(gyroscopeBias().parameterBlocks(time), accelerometerBias().parameterBlocks(time));
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

auto IMU::updateIMUParameterBlockSizes() -> void {
  if (jacobian_type_ == JacobianType::TANGENT_TO_MANIFOLD) {
    parameter_block_sizes_[kGyroscopeIntrinsicsIndex] = GyroscopeIntrinsics::kNumParameters;
    parameter_block_sizes_[kGyroscopeSensitivityIndex] = GyroscopeSensitivity::kNumParameters;
    parameter_block_sizes_[kAccelerometerIntrinsicsIndex] = AccelerometerIntrinsics::kNumParameters;
    parameter_block_sizes_[kAccelerometerOffsetIndex] = AccelerometerOffset::kNumParameters;
  } else {
    parameter_block_sizes_[kGyroscopeIntrinsicsIndex] = variables::Tangent<GyroscopeIntrinsics>::kNumParameters;
    parameter_block_sizes_[kGyroscopeSensitivityIndex] = variables::Tangent<GyroscopeSensitivity>::kNumParameters;
    parameter_block_sizes_[kAccelerometerIntrinsicsIndex] = variables::Tangent<AccelerometerIntrinsics>::kNumParameters;
    parameter_block_sizes_[kAccelerometerOffsetIndex] = variables::Tangent<AccelerometerOffset>::kNumParameters;
  }
}

auto IMU::updateParameterBlockSizes() -> void {
  updateSensorParameterBlockSizes();
  updateIMUParameterBlockSizes();
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

auto IMU::assemblePartitions(GyroscopeBiasParameterBlocks&& gyroscope_bias_parameter_blocks, AccelerometerBiasParameterBlocks&& accelerometer_bias_parameter_blocks) const
    -> Partitions<Scalar*> {
  Partitions<Scalar*> partitions{kNumPartitions};
  partitions[kVariablesPartitionIndex] = assembleVariablesPartition();
  auto& [b_g_offset, b_g_parameter_blocks, b_g_parameter_block_sizes] = partitions[kGyroscopeBiasPartitionIndex];
  auto& [b_a_offset, b_a_parameter_blocks, b_a_parameter_block_sizes] = partitions[kAccelerometerBiasPartitionIndex];
  b_g_offset = kVariablesOffset + static_cast<int>(parameter_blocks_.size());
  b_a_offset = b_g_offset + static_cast<int>(gyroscope_bias_parameter_blocks.size());
  b_g_parameter_block_sizes = std::vector<int>(gyroscope_bias_parameter_blocks.size(), gyroscopeBias().localInputSize());
  b_a_parameter_block_sizes = std::vector<int>(accelerometer_bias_parameter_blocks.size(), accelerometerBias().localInputSize());
  b_g_parameter_blocks = std::move(gyroscope_bias_parameter_blocks);
  b_a_parameter_blocks = std::move(accelerometer_bias_parameter_blocks);
  return partitions;
}

}  // namespace hyper::sensors
