/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/measurement.hpp"
#include "hyper/sensors/sensor.hpp"

namespace hyper::messages {

template <typename TVariable>
class AbsoluteMeasurement final : public MeasurementBase<TVariable> {
 public:
  // Definitions.
  using Variable = TVariable;
  using Base = MeasurementBase<TVariable>;
  using Time = typename Base::Time;

  using Sensor = sensors::Sensor;

  /// Constructor from time, sensor and variable.
  /// \param time Time.
  /// \param sensor Sensor.
  /// \param variable Variable.
  AbsoluteMeasurement(const Time& time, const Sensor& sensor,
                      const TVariable& variable)
      : Base{time, variable}, sensor_{&sensor} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const Sensor& final { *sensor_; }

  /// Sets the associated sensor.
  /// \param sensor Sensor to set.
  inline auto setSensor(const Sensor& sensor) -> void { sensor_ = &sensor; }

 private:
  const Sensor* sensor_;  ///< Sensor.
};

template <typename TManifold>
using ManifoldMeasurement = AbsoluteMeasurement<TManifold>;

template <typename TManifold>
using TangentMeasurement = AbsoluteMeasurement<variables::Tangent<TManifold>>;

}  // namespace hyper::messages
