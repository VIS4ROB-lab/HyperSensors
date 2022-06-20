/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/sensors/sensor.hpp"
#include "hyper/variables/groups/se3.hpp"

namespace hyper {

namespace {

// Parameter names.
constexpr auto kRateName = "rate";
constexpr auto kTransformationName = "transformation";

// Default parameters.
constexpr auto kDefaultRate = -1;

} // namespace

Sensor::Sensor(const Node& node)
    : Sensor{Traits<Sensor>::kNumParameters, node} {}

auto Sensor::rate() const -> const Rate& {
  return rate_;
}

auto Sensor::hasVariableRate() const -> bool {
  return rate() < Scalar{0};
}

auto Sensor::parameters() const -> const Parameters& {
  return parameters_;
}

auto Sensor::memoryBlocks() const -> MemoryBlocks<Scalar> {
  return parameters().memoryBlocks();
}

auto Sensor::memoryBlocks(const Stamp& /* stamp */) const -> MemoryBlocks<Scalar> {
  return parameters().memoryBlocks();
}

auto Sensor::transformation() const -> Eigen::Map<const Transformation> {
  return Eigen::Map<const Transformation>{address(Traits<Sensor>::kTransformationOffset)};
}

auto Sensor::transformation() -> Eigen::Map<Transformation> {
  return Eigen::Map<Transformation>{address(Traits<Sensor>::kTransformationOffset)};
}

auto operator<<(YAML::Emitter& emitter, const Sensor& sensor) -> YAML::Emitter& {
  emitter << YAML::BeginMap;
  sensor.writeParameters(emitter);
  emitter << YAML::EndMap;
  return emitter;
}

Sensor::Sensor(const Size& num_parameters, const Node& node)
    : rate_{kDefaultRate},
      parameters_{num_parameters} {
  initializeParameters();
  if (!node.IsNull()) {
    readParameters(node);
  }
}

auto Sensor::address(const Size& index) const -> const Scalar* {
  return parameters().variable(index).memory().address;
}

auto Sensor::address(const Size& index) -> Scalar* {
  return const_cast<Scalar*>(std::as_const(*this).address(index));
}

auto Sensor::writeParameters(Emitter& emitter) const -> void {
  yaml::Write(emitter, kRateName, rate());
  yaml::WriteVariable(emitter, kTransformationName, transformation());
}

auto Sensor::initializeParameters() -> void {
  DCHECK_LE(Traits<Sensor>::kNumParameters, parameters().variables().size());
  parameters_.setVariable(Traits<Sensor>::kTransformationOffset, std::make_unique<Transformation>());
}

auto Sensor::readParameters(const Node& node) -> void {
  rate_ = yaml::ReadAs<Rate>(node, kRateName);
  transformation() = yaml::ReadVariable<Transformation>(node, kTransformationName);
}

} // namespace hyper
