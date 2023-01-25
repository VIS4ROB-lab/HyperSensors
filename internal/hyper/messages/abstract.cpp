/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/messages/measurements/abstract.hpp"

namespace hyper::messages {

auto AbstractMessage::time() const -> const Time& {
  return time_;
}

auto AbstractMessage::time() -> Time& {
  return const_cast<Time&>(std::as_const(*this).time());
}

auto AbstractMessage::sensor() const -> const Sensor& {
  DCHECK(sensor_ != nullptr);
  return *sensor_;
}

AbstractMessage::AbstractMessage(const Time& time, const Sensor& sensor) : time_{time}, sensor_{&sensor} {}

}  // namespace hyper::messages
