/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#ifdef HYPER_COMPILE_WITH_CERES

#include "hyper/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/sensors/imu.hpp"

namespace hyper::ceres::manifolds {

template <>
class Manifold<sensors::IMU> final : public Manifold<sensors::Sensor> {
 public:
  // Definitions.
  using IMU = sensors::IMU;

  // Constants.
  static constexpr auto kGyroscopeBiasSubmanifoldIndex = IMU::kNumVariables;
  static constexpr auto kAccelerometerBiasSubmanifoldIndex = kGyroscopeBiasSubmanifoldIndex + 1;
  static constexpr auto kNumSubmanifolds = kAccelerometerBiasSubmanifoldIndex + 1;

  /// Default constructor.
  /// \param imu IMU to parametrize.
  /// \param constant Constancy flag.
  explicit Manifold(IMU* imu, bool constant = true);

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const IMU* final;

  /// Gyroscope intrinsics submanifold accessor.
  [[nodiscard]] auto gyroscopeIntrinsicsSubmanifold() const -> Submanifold*;

  /// Sets the gyroscope intrinsics submanifold.
  /// \param submanifold Input submanifold.
  auto setGyroscopeIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the sensor's gyroscope intrinsics constant or variable.
  /// \param constant Constancy flag.
  auto setGyroscopeIntrinsicsConstant(bool constant) -> void;

  /// Gyroscope sensitivity submanifold accessor.
  [[nodiscard]] auto gyroscopeSensitivitySubmanifold() const -> Submanifold*;

  /// Sets the gyroscope sensitivity submanifold.
  /// \param submanifold Input submanifold.
  auto setGyroscopeSensitivitySubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the sensor's gyroscope sensitivity constant or variable.
  /// \param constant Constancy flag.
  auto setGyroscopeSensitivityConstant(bool constant) -> void;

  /// Gyroscope bias submanifold accessor.
  [[nodiscard]] auto gyroscopeBiasSubmanifold() const -> Submanifold*;

  /// Sets the gyroscope bias submanifold.
  /// \param submanifold Input submanifold.
  auto setGyroscopeBiasSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the sensor's gyroscope bias constant or variable.
  /// \param constant Constancy flag.
  auto setGyroscopeBiasConstant(bool constant) -> void;

  /// Accelerometer intrinsics submanifold accessor.
  [[nodiscard]] auto accelerometerIntrinsicsSubmanifold() const -> Submanifold*;

  /// Sets the accelerometer intrinsics submanifold.
  /// \param submanifold Input submanifold.
  auto setAccelerometerIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the sensor's accelerometer intrinsics constant or variable.
  /// \param constant Constancy flag.
  auto setAccelerometerIntrinsicsConstant(bool constant) -> void;

  /// Accelerometer offset submanifold accessor.
  [[nodiscard]] auto accelerometerOffsetSubmanifold() const -> Submanifold*;

  /// Sets the accelerometer offset submanifold.
  /// \param submanifold Input submanifold.
  auto setAccelerometerOffsetSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the sensor's accelerometer offset constant or variable.
  /// \param constant Constancy flag.
  auto setAccelerometerOffsetConstant(bool constant) -> void;

  /// Accelerometer bias submanifold accessor.
  [[nodiscard]] auto accelerometerBiasSubmanifold() const -> Submanifold*;

  /// Sets the accelerometer bias submanifold.
  /// \param submanifold Input submanifold.
  auto setAccelerometerBiasSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the sensor's accelerometer bias constant or variable.
  /// \param constant Constancy flag.
  auto setAccelerometerBiasConstant(bool constant) -> void;
};

}  // namespace hyper::ceres::manifolds

#endif
