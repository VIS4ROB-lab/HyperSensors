/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/variable.hpp"

namespace hyper::messages {

template <typename TSensor, typename TVariable>
class RelativeMeasurement final : public VariableMeasurement<TVariable> {
 public:
  // Definitions.
  using Sensor = TSensor;
  using Variable = TVariable;
  using Base = VariableMeasurement<TVariable>;
  using Time = typename Base::Time;

  /// Constructor from time, sensor and variable.
  /// \param time Time.
  /// \param sensor Sensor.
  /// \param other_time Other time.
  /// \param other_sensor Other sensor.
  /// \param variable Variable.
  RelativeMeasurement(const Time& time, const Sensor& sensor,
                      const Time& other_time, const Sensor& other_sensor,
                      const TVariable& variable)
      : VariableMeasurement<TVariable>{time, variable},
        other_time_{other_time},
        sensor_{&sensor},
        other_sensor_{&other_sensor} {}

  /// Other time accessor.
  /// \return Other time.
  [[nodiscard]] auto otherTime() const -> const Time& { return other_time_; }

  /// Other time modifier.
  /// \return Other time.
  auto otherTime() -> Time& {
    return static_cast<Time&>(std::as_const(*this).otherTime());
  }

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const Sensor& final { *sensor_; }

  /// Sets the associated sensor.
  /// \param camera Sensor to set.
  inline auto setSensor(const Sensor& sensor) -> void { sensor_ = &sensor; }

  /// Other sensor accessor.
  /// \return Other sensor.
  [[nodiscard]] inline auto otherSensor() const -> const Sensor& {
    return *other_sensor_;
  }

  /// Sets the associated other sensor.
  /// \param camera Other sensor to set.
  inline auto setOtherSensor(const Sensor& other_sensor) -> void {
    other_sensor_ = &other_sensor;
  }

 private:
  Time other_time_;              ///< Other time.
  const TSensor* sensor_;        ///< Sensor.
  const TSensor* other_sensor_;  ///< Other sensor.
};

template <typename TSensor, typename TManifold>
using RelativeManifoldMeasurement = RelativeMeasurement<TSensor, TManifold>;

template <typename TSensor, typename TManifold>
using RelativeTangentMeasurement =
    RelativeMeasurement<TSensor, variables::Tangent<TManifold>>;

}  // namespace hyper::messages
