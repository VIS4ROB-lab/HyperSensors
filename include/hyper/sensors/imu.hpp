/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/sensor.hpp"
#include "hyper/state/abstract.hpp"
#include "hyper/variables/orthonormality_alignment.hpp"

namespace hyper {

class IMU final : public Sensor {
 public:
  // Definitions.
  using GyroscopeNoiseDensity = Traits<IMU>::GyroscopeNoiseDensity;
  using GyroscopeIntrinsics = Traits<IMU>::GyroscopeIntrinsics;
  using GyroscopeSensitivity = Traits<IMU>::GyroscopeSensitivity;
  using GyroscopeBias = Traits<IMU>::GyroscopeBias;

  using AccelerometerNoiseDensity = Traits<IMU>::AccelerometerNoiseDensity;
  using AccelerometerIntrinsics = Traits<IMU>::AccelerometerIntrinsics;
  using AccelerometerAxesOffsets = Traits<IMU>::AccelerometerAxesOffsets;
  using AccelerometerBias = Traits<IMU>::AccelerometerBias;

  /// Constructor from YAML file.
  /// \param node Input YAML node.
  explicit IMU(const Node& node = {});

  /// Collects the memory blocks.
  /// \return Memory blocks.
  [[nodiscard]] auto memoryBlocks() const -> MemoryBlocks<Scalar> final;

  /// Collects the (time-specific) memory blocks.
  /// \param stamp Query stamp.
  /// \return Memory blocks.
  [[nodiscard]] auto memoryBlocks(const Stamp& stamp) const -> MemoryBlocks<Scalar> final;

  /// \brief Gyroscope noise density accessor.
  /// \return Gyroscope noise density.
  [[nodiscard]] auto gyroscopeNoiseDensity() const -> const GyroscopeNoiseDensity&;

  /// \brief Gyroscope noise density modifier.
  /// \return Gyroscope noise density.
  auto gyroscopeNoiseDensity() -> GyroscopeNoiseDensity&;

  /// Gyroscope bias accessor.
  /// \return Gyroscope bias.
  [[nodiscard]] auto gyroscopeBias() const -> const AbstractState&;

  /// Gyroscope bias modifier.
  /// \return Gyroscope bias.
  auto gyroscopeBias() -> AbstractState&;

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
  [[nodiscard]] auto accelerometerBias() const -> const AbstractState&;

  /// Accelerometer bias modifier.
  /// \return Accelerometer bias.
  auto accelerometerBias() -> AbstractState&;

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
  /// Initializes the parameters.
  auto initializeParameters() -> void;

  /// Reads all sensor parameters from a YAML node.
  /// \param node Input YAML node.
  auto readParameters(const Node& node) -> void;

  /// Outputs all sensor parameters to a YAML emitter.
  /// \param emitter Output YAML emitter.
  auto writeParameters(Emitter& emitter) const -> void final;

  GyroscopeNoiseDensity gyroscope_noise_density_;         ///< Gyroscope noise density.
  std::unique_ptr<AbstractState> gyroscope_bias_;         ///< Gyroscope bias.
  AccelerometerNoiseDensity accelerometer_noise_density_; ///< Accelerometer noise density.
  std::unique_ptr<AbstractState> accelerometer_bias_;     ///< Accelerometer bias.
};

} // namespace hyper
