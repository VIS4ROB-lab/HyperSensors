/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/messages/measurements/abstract.hpp"

namespace hyper::messages {

AbstractMeasurement::AbstractMeasurement(const Time& time, const Sensor& sensor) : AbstractMessage{time, sensor} {}

}  // namespace hyper::messages
