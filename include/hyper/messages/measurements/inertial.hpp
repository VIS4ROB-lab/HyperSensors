/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/variable.hpp"
#include "hyper/sensors/imu.hpp"

namespace hyper::messages {

template <typename TManifold>
class InertialMeasurement final : public TangentMeasurement<TManifold> {
 public:
  // Definitions.
  using Manifold = TManifold;
  using Base = TangentMeasurement<TManifold>;
  using Time = typename Base::Time;

  using IMU = sensors::IMU;
  using Tangent = variables::Tangent<TManifold>;

  /// Constructor from time, sensor and value.
  /// \param time Time.
  /// \param imu Sensor.
  /// \param tangent Tangent.
  InertialMeasurement(const Time& time, const IMU& imu, const Tangent& tangent)
      : TangentMeasurement<TManifold>{time, tangent}, imu_{&imu} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const IMU& final { *imu_; }

  /// Sets the associated sensor.
  /// \param imu Sensor to set.
  inline auto setSensor(const IMU& imu) -> void { imu_ = &imu; }

 private:
  const IMU* imu_;  ///< IMU.
};

}  // namespace hyper::messages
