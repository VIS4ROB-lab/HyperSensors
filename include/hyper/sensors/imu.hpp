/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/sensor.hpp"
#include "hyper/state/continuous.hpp"
#include "hyper/variables/orthonormality_alignment.hpp"

namespace hyper::sensors {

class IMU final : public Sensor {
 public:
  // Constants.
  static constexpr auto kGyroscopeIntrinsicsOffset = Sensor::kNumParameters;
  static constexpr auto kAccelerometerIntrinsicsOffset = kGyroscopeIntrinsicsOffset + 1;
  static constexpr auto kAccelerometerAxesOffsetsOffset = kAccelerometerIntrinsicsOffset + 1;
  static constexpr auto kGyroscopeSensitivityOffset = kAccelerometerAxesOffsetsOffset + 1;
  static constexpr auto kNumParameters = kGyroscopeSensitivityOffset + 1;

  // Definitions.
  using GyroscopeNoiseDensity = Scalar;
  using GyroscopeIntrinsics = variables::OrthonormalityAlignment<Scalar, 3>;
  using GyroscopeSensitivity = variables::Cartesian<Scalar, 9>;
  using GyroscopeBias = state::ContinuousState<variables::Cartesian<Scalar, 3>>;

  using AccelerometerNoiseDensity = Scalar;
  using AccelerometerIntrinsics = variables::OrthonormalityAlignment<Scalar, 3>;
  using AccelerometerAxesOffsets = variables::Cartesian<Scalar, 9>;
  using AccelerometerBias = state::ContinuousState<variables::Cartesian<Scalar, 3>>;

  /// Constructor from YAML file.
  /// \param node Input YAML node.
  explicit IMU(const Node& node = {});

  /// Pointers accessor.
  /// \return Pointers.
  [[nodiscard]] auto pointers() const -> std::vector<Variable*> final;

  /// Pointers accessor (time-based).
  /// \param time Query time.
  /// \return Pointers.
  [[nodiscard]] auto pointers(const Time& time) const -> std::vector<Variable*> final;

  /// \brief Gyroscope noise density accessor.
  /// \return Gyroscope noise density.
  [[nodiscard]] auto gyroscopeNoiseDensity() const -> const GyroscopeNoiseDensity&;

  /// \brief Gyroscope noise density modifier.
  /// \return Gyroscope noise density.
  auto gyroscopeNoiseDensity() -> GyroscopeNoiseDensity&;

  /// Gyroscope bias accessor.
  /// \return Gyroscope bias.
  [[nodiscard]] auto gyroscopeBias() const -> const GyroscopeBias&;

  /// Gyroscope bias modifier.
  /// \return Gyroscope bias.
  auto gyroscopeBias() -> GyroscopeBias&;

  /// Gyroscope intrinsics accessor.
  /// \return Gyroscope intrinsics.
  [[nodiscard]] auto gyroscopeIntrinsics() const -> Eigen::Map<const GyroscopeIntrinsics>;

  /// Gyroscope intrinsics modifier.
  /// \return Gyroscope intrinsics.
  auto gyroscopeIntrinsics() -> Eigen::Map<GyroscopeIntrinsics>;

  /// Gyroscope sensitivity accessor.
  /// \return Gyroscope sensitivity.
  [[nodiscard]] auto gyroscopeSensitivity() const -> Eigen::Map<const GyroscopeSensitivity>;

  /// Gyroscope sensitivity modifier.
  /// \return Gyroscope sensitivity.
  auto gyroscopeSensitivity() -> Eigen::Map<GyroscopeSensitivity>;

  /// \brief Accelerometer noise density accessor.
  /// \return Accelerometer noise density.
  [[nodiscard]] auto accelerometerNoiseDensity() const -> const AccelerometerNoiseDensity&;

  /// \brief Accelerometer noise density modifier.
  /// \return Accelerometer noise density.
  auto accelerometerNoiseDensity() -> AccelerometerNoiseDensity&;

  /// Accelerometer bias accessor.
  /// \return Accelerometer bias.
  [[nodiscard]] auto accelerometerBias() const -> const AccelerometerBias&;

  /// Accelerometer bias modifier.
  /// \return Accelerometer bias.
  auto accelerometerBias() -> AccelerometerBias&;

  /// Accelerometer intrinsics accessor.
  /// \return Accelerometer intrinsics.
  [[nodiscard]] auto accelerometerIntrinsics() const -> Eigen::Map<const AccelerometerIntrinsics>;

  /// Accelerometer intrinsics modifier.
  /// \return Accelerometer intrinsics.
  auto accelerometerIntrinsics() -> Eigen::Map<AccelerometerIntrinsics>;

  /// Accelerometer axes offsets accessor.
  /// \return Accelerometer axes offsets.
  [[nodiscard]] auto accelerometerAxesOffsets() const -> Eigen::Map<const AccelerometerAxesOffsets>;

  /// Accelerometer axes offsets modifier.
  /// \return Accelerometer axes offsets.
  auto accelerometerAxesOffsets() -> Eigen::Map<AccelerometerAxesOffsets>;

 private:
  /// Initializes the variables.
  auto initializeVariables() -> void;

  /// Reads all sensor variables from a YAML node.
  /// \param node Input YAML node.
  auto readVariables(const Node& node) -> void;

  /// Outputs all sensor variables to a YAML emitter.
  /// \param emitter Output YAML emitter.
  auto writeVariables(Emitter& emitter) const -> void final;

  GyroscopeNoiseDensity gyroscope_noise_density_;          ///< Gyroscope noise density.
  GyroscopeBias gyroscope_bias_;                           ///< Gyroscope bias.
  AccelerometerNoiseDensity accelerometer_noise_density_;  ///< Accelerometer noise density.
  AccelerometerBias accelerometer_bias_;                   ///< Accelerometer bias.
};

}  // namespace hyper::sensors
