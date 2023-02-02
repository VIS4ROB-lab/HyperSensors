/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/variables/forward.hpp"

#include "hyper/messages/measurements/measurement.hpp"

namespace hyper::messages {

template <typename TVariable>
class VariableMeasurement : public Measurement<typename TVariable::Scalar> {
 public:
  // Definitions.
  using Variable = TVariable;
  using Base = Measurement<typename TVariable::Scalar>;
  using Time = typename Base::Time;

  /// Constructor from time and variable.
  /// \param time Time.
  /// \param variable Variable.
  VariableMeasurement(const Time& time, const TVariable& variable)
      : Base{time}, variable_{variable} {}

  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] inline auto variable() const -> const TVariable& final {
    return variable_;
  }

  /// Variable modifier.
  /// \return Variable.
  inline auto variable() -> TVariable& final {
    return const_cast<TVariable&>(std::as_const(*this).variable());
  }

 private:
  TVariable variable_;  ///< Variable.
};

template <typename TManifold>
using AbsoluteMeasurement = VariableMeasurement<TManifold>;

template <typename TManifold>
using ManifoldMeasurement = VariableMeasurement<TManifold>;

template <typename TManifold>
using TangentMeasurement = VariableMeasurement<variables::Tangent<TManifold>>;

}  // namespace hyper::messages
