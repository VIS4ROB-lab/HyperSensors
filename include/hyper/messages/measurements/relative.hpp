/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/measurement.hpp"

namespace hyper::messages {

template <typename TSensor, typename TValue>
class RelativeMeasurement final : public MeasurementBase<TValue> {
 public:
  // Definitions.
  using Base = MeasurementBase<TValue>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  /// Constructor from time, sensor and value.
  /// \param time Time.
  /// \param sensor Sensor.
  /// \param other_time Other time.
  /// \param other_sensor Other sensor.
  /// \param value Value.
  RelativeMeasurement(const Time& time, const TSensor& sensor, const Time& other_time, const TSensor& other_sensor, const TValue& value)
      : Base{Type::RELATIVE_MEASUREMENT, time, value}, other_time_{other_time}, sensor_{&sensor}, other_sensor_{&other_sensor} {}

  /// Other time accessor.
  /// \return Other time.
  [[nodiscard]] auto otherTime() const -> const Time& { return other_time_; }

  /// Other time modifier.
  /// \return Other time.
  auto otherTime() -> Time& { return static_cast<Time&>(std::as_const(*this).otherTime()); }

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const TSensor& final { *sensor_; }

  /// Sets the associated sensor.
  /// \param camera Sensor to set.
  inline auto setSensor(const TSensor& sensor) -> void { sensor_ = &sensor; }

  /// Other sensor accessor.
  /// \return Other sensor.
  [[nodiscard]] inline auto otherSensor() const -> const TSensor& { return *other_sensor_; }

  /// Sets the associated other sensor.
  /// \param camera Other sensor to set.
  inline auto setOtherSensor(const TSensor& other_sensor) -> void { other_sensor_ = &other_sensor; }

 private:
  Time other_time_;              ///< Other time.
  const TSensor* sensor_;        ///< Sensor.
  const TSensor* other_sensor_;  ///< Other sensor.
};

}  // namespace hyper::messages
