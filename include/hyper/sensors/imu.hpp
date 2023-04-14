/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/sensor.hpp"
#include "hyper/state/continuous.hpp"
#include "hyper/variables/orthonormality_alignment.hpp"
#include "hyper/variables/sensitivity.hpp"

namespace hyper::sensors {

class IMU final : public Sensor {
 public:
  // Constants.
  static constexpr auto kGyroscopeBiasPartitionIndex = Sensor::kNumPartitions;
  static constexpr auto kAccelerometerBiasPartitionIndex = kGyroscopeBiasPartitionIndex + 1;
  static constexpr auto kNumPartitions = kAccelerometerBiasPartitionIndex + 1;

  static constexpr auto kGyroscopeIntrinsicsIndex = Sensor::kNumVariables;
  static constexpr auto kGyroscopeSensitivityIndex = kGyroscopeIntrinsicsIndex + 1;
  static constexpr auto kAccelerometerIntrinsicsIndex = kGyroscopeSensitivityIndex + 1;
  static constexpr auto kAccelerometerOffsetIndex = kAccelerometerIntrinsicsIndex + 1;
  static constexpr auto kNumVariables = kAccelerometerOffsetIndex + 1;

  // Definitions.
  using GyroscopeNoiseDensity = Scalar;
  using GyroscopeIntrinsics = variables::OrthonormalityAlignment<Scalar, 3>;
  using GyroscopeSensitivity = variables::Sensitivity<Scalar, 3>;
  using GyroscopeBias = state::ContinuousState<variables::Cartesian<Scalar, 3>>;

  using AccelerometerNoiseDensity = Scalar;
  using AccelerometerIntrinsics = variables::OrthonormalityAlignment<Scalar, 3>;
  using AccelerometerOffset = variables::Cartesian<Scalar, 9>;
  using AccelerometerBias = state::ContinuousState<variables::Cartesian<Scalar, 3>>;

  using BiasInterpolator = state::TemporalInterpolator<Scalar>;
  using DefaultBiasInterpolator = state::BasisInterpolator<Scalar, 4>;

  /// Constructor from Jacobian type and bias interpolators.
  /// \param jacobian_type Jacobian type.
  /// \param gyroscope_bias_interpolator Interpolator.
  /// \param accelerometer_bias_interpolator Interpolator.
  explicit IMU(JacobianType jacobian_type = kDefaultJacobianType, std::unique_ptr<BiasInterpolator>&& gyroscope_bias_interpolator = std::make_unique<DefaultBiasInterpolator>(),
               std::unique_ptr<BiasInterpolator>&& accelerometer_bias_interpolator = std::make_unique<DefaultBiasInterpolator>());

  /// Constructor from YAML node.
  /// \param node YAML node.
  explicit IMU(const Node& node);

  /// \brief Gyroscope noise density accessor.
  /// \return Gyroscope noise density.
  [[nodiscard]] auto gyroscopeNoiseDensity() const -> const GyroscopeNoiseDensity&;

  /// \brief Gyroscope noise density modifier.
  /// \return Gyroscope noise density.
  auto gyroscopeNoiseDensity() -> GyroscopeNoiseDensity&;

  /// Gyroscope intrinsics accessor.
  /// \return Gyroscope intrinsics.
  [[nodiscard]] auto gyroscopeIntrinsics() const -> const GyroscopeIntrinsics&;

  /// Gyroscope intrinsics modifier.
  /// \return Gyroscope intrinsics.
  auto gyroscopeIntrinsics() -> GyroscopeIntrinsics&;

  /// Gyroscope sensitivity accessor.
  /// \return Gyroscope sensitivity.
  [[nodiscard]] auto gyroscopeSensitivity() const -> const GyroscopeSensitivity&;

  /// Gyroscope sensitivity modifier.
  /// \return Gyroscope sensitivity.
  auto gyroscopeSensitivity() -> GyroscopeSensitivity&;

  /// Gyroscope bias accessor.
  /// \return Gyroscope bias.
  [[nodiscard]] auto gyroscopeBias() const -> const GyroscopeBias&;

  /// Gyroscope bias modifier.
  /// \return Gyroscope bias.
  auto gyroscopeBias() -> GyroscopeBias&;

  /// \brief Accelerometer noise density accessor.
  /// \return Accelerometer noise density.
  [[nodiscard]] auto accelerometerNoiseDensity() const -> const AccelerometerNoiseDensity&;

  /// \brief Accelerometer noise density modifier.
  /// \return Accelerometer noise density.
  auto accelerometerNoiseDensity() -> AccelerometerNoiseDensity&;

  /// Accelerometer intrinsics accessor.
  /// \return Accelerometer intrinsics.
  [[nodiscard]] auto accelerometerIntrinsics() const -> const AccelerometerIntrinsics&;

  /// Accelerometer intrinsics modifier.
  /// \return Accelerometer intrinsics.
  auto accelerometerIntrinsics() -> AccelerometerIntrinsics&;

  /// Accelerometer offset (of individual axes) accessor.
  /// \return Accelerometer offset.
  [[nodiscard]] auto accelerometerOffset() const -> const AccelerometerOffset&;

  /// Accelerometer offset (of individual axes) modifier.
  /// \return Accelerometer offset.
  auto accelerometerOffset() -> AccelerometerOffset&;

  /// Accelerometer bias accessor.
  /// \return Accelerometer bias.
  [[nodiscard]] auto accelerometerBias() const -> const AccelerometerBias&;

  /// Accelerometer bias modifier.
  /// \return Accelerometer bias.
  auto accelerometerBias() -> AccelerometerBias&;

  /// Time-based parameter blocks accessor.
  /// \return Time-based pointers to parameter blocks.
  [[nodiscard]] auto partitions(const Time& time) const -> variables::Partitions<Scalar*> final;

 private:
  // Definitions.
  using GyroscopeBiasParameterBlocks = std::vector<Scalar*>;
  using AccelerometerBiasParameterBlocks = std::vector<Scalar*>;

  /// Updates the IMU parameter block sizes.
  auto updateIMUParameterBlockSizes() -> void;

  /// Updates the parameter block sizes.
  auto updateParameterBlockSizes() -> void final;

  /// Reads a sensor from a YAML node.
  /// \param node YAML node.
  auto read(const Node& node) -> void final;

  /// Writes a sensor to a YAML emitter.
  /// \param emitter YAML emitter.
  auto write(Emitter& emitter) const -> void final;

  GyroscopeNoiseDensity gyroscope_noise_density_;          ///< Gyroscope noise density.
  GyroscopeBias gyroscope_bias_;                           ///< Gyroscope bias.
  AccelerometerNoiseDensity accelerometer_noise_density_;  ///< Accelerometer noise density.
  AccelerometerBias accelerometer_bias_;                   ///< Accelerometer bias.
};

}  // namespace hyper::sensors
