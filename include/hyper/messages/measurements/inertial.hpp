/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/variable.hpp"
#include "hyper/sensors/imu.hpp"

namespace hyper::messages {

template <typename TManifold>
class InertialMeasurement final
    : public TangentMeasurement<TManifold> {
 public:
  /// Constructor from stamp, sensor and value.
  /// \param stamp Stamp.
  /// \param imu Sensor.
  /// \param tangent Tangent.
  InertialMeasurement(const Stamp& stamp, const IMU& imu, const Tangent<TManifold>& tangent)
      : TangentMeasurement<TManifold>{stamp, imu, tangent} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const IMU& final {
    return static_cast<const IMU&>(*this->sensor_); // NOLINT
  }

  /// Sets the associated sensor.
  /// \param imu Sensor to set.
  auto setSensor(const IMU& imu) -> void {
    this->sensor_ = &imu;
  }
};

} // namespace hyper::messages
