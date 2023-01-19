/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/abstract.hpp"
#include "hyper/variables/abstract.hpp"

namespace hyper::messages {

class AbstractMeasurement : public AbstractMessage {
 public:
  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] virtual auto variable() const -> const AbstractVariable<Scalar>& = 0;

  /// Variable modifier.
  /// \return Variable.
  [[nodiscard]] virtual auto variable() -> AbstractVariable<Scalar>& = 0;

 protected:
  /// Constructor from stamp and sensor.
  /// \param stamp Stamp.
  /// \param sensor Sensor.
  explicit AbstractMeasurement(const Stamp& stamp, const Sensor& sensor);
};

}  // namespace hyper::messages
