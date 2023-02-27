/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/measurement.hpp"
#include "hyper/sensors/sensor.hpp"

namespace hyper::messages {

template <typename TValue>
class AbsoluteMeasurement final : public MeasurementBase<TValue> {
 public:
  // Definitions.
  using Base = MeasurementBase<TValue>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  using Sensor = sensors::Sensor;

  /// Constructor from time, sensor and value.
  /// \param time Time.
  /// \param sensor Sensor.
  /// \param value Value.
  AbsoluteMeasurement(const Time& time, const Sensor* sensor, const TValue& value) : Base{Type::ABSOLUTE_MEASUREMENT, time, value}, sensor_{sensor} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const Sensor* final { return sensor_; }

  /// Sets the associated sensor.
  /// \param sensor Sensor to set.
  inline auto setSensor(const Sensor* sensor) -> void { sensor_ = sensor; }

 private:
  const Sensor* sensor_;  ///< Sensor.
};

template <typename TManifold>
using ManifoldMeasurement = AbsoluteMeasurement<TManifold>;

template <typename TManifold>
using TangentMeasurement = AbsoluteMeasurement<variables::Tangent<TManifold>>;

}  // namespace hyper::messages
