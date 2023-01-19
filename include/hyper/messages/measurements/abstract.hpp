/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/abstract.hpp"
#include "hyper/variables/variable.hpp"

namespace hyper::messages {

class AbstractMeasurement : public AbstractMessage {
 public:
  // Definitions.
  using Variable = variables::Variable<Scalar>;

  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] virtual auto variable() const -> const Variable& = 0;

  /// Variable modifier.
  /// \return Variable.
  [[nodiscard]] virtual auto variable() -> Variable& = 0;

 protected:
  /// Constructor from time and sensor.
  /// \param time Time.
  /// \param sensor Sensor.
  explicit AbstractMeasurement(const Time& time, const Sensor& sensor);
};

}  // namespace hyper::messages
