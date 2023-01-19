/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/messages/measurements/abstract.hpp"

namespace hyper::messages {

auto AbstractMessage::stamp() const -> const Stamp& {
  return stamp_;
}

auto AbstractMessage::stamp() -> Stamp& {
  return const_cast<Stamp&>(std::as_const(*this).stamp());
}

auto AbstractMessage::sensor() const -> const Sensor& {
  DCHECK(sensor_ != nullptr);
  return *sensor_;
}

AbstractMessage::AbstractMessage(const Stamp& stamp, const Sensor& sensor) : stamp_{stamp}, sensor_{&sensor} {}

}  // namespace hyper::messages
