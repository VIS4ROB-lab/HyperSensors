/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/variable.hpp"

namespace hyper::messages {

template <typename TVariable>
class RelativeMeasurement final : public VariableMeasurement<TVariable> {
 public:
  /// Constructor from stamp, sensor and variable.
  /// \param stamp Stamp.
  /// \param sensor Sensor.
  /// \param other_stamp Other stamp.
  /// \param other_sensor Other sensor.
  /// \param variable Variable.
  RelativeMeasurement(const Stamp& stamp, const Sensor& sensor, const Stamp& other_stamp, const Sensor& other_sensor, const TVariable& variable)
      : VariableMeasurement<TVariable>{stamp, sensor, variable}, other_stamp_{other_stamp}, other_sensor_{&other_sensor} {}

  /// Other stamp accessor.
  /// \return Other stamp.
  [[nodiscard]] auto otherStamp() const -> const Stamp& { return other_stamp_; }

  /// Other stamp modifier.
  /// \return Other stamp.
  auto otherStamp() -> Stamp& { return static_cast<Stamp&>(std::as_const(*this).otherStamp()); }

  /// Other sensor accessor.
  /// \return Other sensor.
  [[nodiscard]] auto otherSensor() const -> const Sensor& { return *other_sensor_; }

 private:
  Stamp other_stamp_;           ///< Other stamp.
  const Sensor* other_sensor_;  ///< Other sensor.
};

template <typename TManifold>
using RelativeManifoldMeasurement = RelativeMeasurement<TManifold>;

template <typename TManifold>
using RelativeTangentMeasurement = RelativeMeasurement<Tangent<TManifold>>;

}  // namespace hyper::messages
