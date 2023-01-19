/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/messages/measurements/abstract.hpp"

namespace hyper::messages {

AbstractMeasurement::AbstractMeasurement(const Stamp& stamp, const Sensor& sensor) : AbstractMessage{stamp, sensor} {}

}  // namespace hyper::messages
