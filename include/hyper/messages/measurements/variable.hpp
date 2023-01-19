/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/variables/forward.hpp"

#include "hyper/messages/measurements/abstract.hpp"

namespace hyper::messages {

template <typename TVariable>
class VariableMeasurement : public AbstractMeasurement {
 public:
  /// Constructor from stamp, sensor and variable.
  /// \param stamp Stamp.
  /// \param sensor Sensor.
  /// \param variable Variable.
  VariableMeasurement(const Stamp& stamp, const Sensor& sensor, const TVariable& variable) : AbstractMeasurement{stamp, sensor}, variable_{variable} {}

  /// Sets the associated sensor.
  /// \param sensor Sensor to set.
  auto setSensor(const Sensor& sensor) -> void { sensor_ = &sensor; }

  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] auto variable() const -> const TVariable& final { return variable_; }

  /// Variable modifier.
  /// \return Variable.
  auto variable() -> TVariable& final { return const_cast<TVariable&>(std::as_const(*this).variable()); }

 private:
  TVariable variable_;  ///< Variable.
};

template <typename TManifold>
using ManifoldMeasurement = VariableMeasurement<TManifold>;

template <typename TManifold>
using TangentMeasurement = VariableMeasurement<Tangent<TManifold>>;

}  // namespace hyper::messages
