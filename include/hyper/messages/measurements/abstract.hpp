/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/abstract.hpp"

namespace hyper {

class AbstractMeasurement
    : public AbstractMessage {
 public:
  /// Memory block accessor.
  /// \return Memory block.
  [[nodiscard]] virtual auto memoryBlock() const -> MemoryBlock<const Scalar> = 0;

 protected:
  /// Constructor from stamp and sensor.
  /// \param stamp Stamp.
  /// \param sensor Sensor.
  explicit AbstractMeasurement(const Stamp& stamp, const Sensor& sensor);
};

} // namespace hyper
