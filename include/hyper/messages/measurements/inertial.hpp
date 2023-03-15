/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/measurement.hpp"
#include "hyper/sensors/imu.hpp"

namespace hyper::messages {

template <typename TManifold>
class InertialMeasurement final : public MeasurementBase<variables::Tangent<TManifold>> {
 public:
  // Definitions.
  using Base = MeasurementBase<variables::Tangent<TManifold>>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;
  using Value = typename Base::Value;

  using Sensor = sensors::IMU;

  /// Constructor from time, sensor and value.
  /// \param time Time.
  /// \param sensor Sensor.
  /// \param value Value.
  InertialMeasurement(const Time& time, const Sensor* sensor, const Value& value) : Base{Type::INERTIAL_MEASUREMENT, time, value}, sensor_{sensor} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const Sensor* final { return sensor_; }

  /// Sets the associated sensor.
  /// \param sensor Sensor to set.
  inline auto setSensor(const Sensor* sensor) -> void { sensor_ = sensor; }

 private:
  const Sensor* sensor_;  ///< Sensor.
};

}  // namespace hyper::messages
