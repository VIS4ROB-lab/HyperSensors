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
  static constexpr auto kGyroscopeIntrinsicsOffset = Sensor::kNumVariables;
  static constexpr auto kGyroscopeSensitivityOffset = kGyroscopeIntrinsicsOffset + 1;
  static constexpr auto kAccelerometerIntrinsicsOffset = kGyroscopeSensitivityOffset + 1;
  static constexpr auto kAccelerometerAxesOffsetsOffset = kAccelerometerIntrinsicsOffset + 1;
  static constexpr auto kNumVariables = kAccelerometerAxesOffsetsOffset + 1;

  // Definitions.
  using GyroscopeNoiseDensity = Scalar;
  using GyroscopeIntrinsics = variables::OrthonormalityAlignment<Scalar, 3>;
  using GyroscopeSensitivity = variables::Sensitivity<Scalar, 3>;
  using GyroscopeBias = state::ContinuousState<variables::Cartesian<Scalar, 3>>;

  using AccelerometerNoiseDensity = Scalar;
  using AccelerometerIntrinsics = variables::OrthonormalityAlignment<Scalar, 3>;
  using AccelerometerAxesOffsets = variables::Cartesian<Scalar, 9>;
  using AccelerometerBias = state::ContinuousState<variables::Cartesian<Scalar, 3>>;

  /// Default constructor.
  IMU();

  /// Variable pointers accessor.
  /// \return Pointers to variables.
  [[nodiscard]] auto variables() const -> std::vector<Variable*> final;

  /// Time-based variable pointers accessor.
  /// \return Time-based pointers to variables.
  [[nodiscard]] auto variables(const Time& time) const -> std::vector<Variable*> final;

  /// Parameter blocks accessor.
  /// \return Pointers to parameter blocks.
  [[nodiscard]] auto parameterBlocks() const -> std::vector<Scalar*> final;

  /// Time-based parameter blocks accessor.
  /// \return Time-based pointers to parameter blocks.
  [[nodiscard]] auto parameterBlocks(const Time& time) const -> std::vector<Scalar*> final;

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

  /// Accelerometer axes offsets accessor.
  /// \return Accelerometer axes offsets.
  [[nodiscard]] auto accelerometerAxesOffsets() const -> const AccelerometerAxesOffsets&;

  /// Accelerometer axes offsets modifier.
  /// \return Accelerometer axes offsets.
  auto accelerometerAxesOffsets() -> AccelerometerAxesOffsets&;

  /// Accelerometer bias accessor.
  /// \return Accelerometer bias.
  [[nodiscard]] auto accelerometerBias() const -> const AccelerometerBias&;

  /// Accelerometer bias modifier.
  /// \return Accelerometer bias.
  auto accelerometerBias() -> AccelerometerBias&;

 private:
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
