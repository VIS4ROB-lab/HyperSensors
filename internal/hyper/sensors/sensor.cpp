/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/sensors/sensor.hpp"
#include "hyper/variables/groups/se3.hpp"

namespace hyper::sensors {

namespace {

// Variable names.
constexpr auto kRateName = "rate";
constexpr auto kOffsetName = "offset";
constexpr auto kTransformationName = "transformation";

// Default parameters.
constexpr auto kDefaultRate = -1;

}  // namespace

Sensor::Sensor(const Node& node) : Sensor{kNumVariables, node} {}

auto Sensor::rate() const -> const Rate& {
  return rate_;
}

auto Sensor::hasVariableRate() const -> bool {
  return rate() < Scalar{0};
}

auto Sensor::variables() const -> const Variables& {
  return variables_;
}

auto Sensor::pointers() const -> std::vector<Variable*> {
  std::vector<Variable*> pointers;
  pointers.reserve(variables_.size());
  std::transform(variables_.begin(), variables_.end(), std::back_inserter(pointers), [](const auto& arg) { return arg.get(); });
  return pointers;
}

auto Sensor::pointers(const Time& /* time */) const -> std::vector<Variable*> {
  return Sensor::pointers();
}

auto Sensor::parameters() const -> std::vector<Scalar*> {
  return parameters_;
}

auto Sensor::parameters(const Time& /* time */) const -> std::vector<Scalar*> {
  return Sensor::parameters();
}

auto Sensor::offset() const -> Eigen::Map<const Offset> {
  return Eigen::Map<const Offset>{parameters_[kOffsetOffset]};
}

auto Sensor::offset() -> Eigen::Map<Offset> {
  return Eigen::Map<Offset>{parameters_[kOffsetOffset]};
}

auto Sensor::transformation() const -> Eigen::Map<const Transformation> {
  return Eigen::Map<const Transformation>{parameters_[kTransformationOffset]};
}

auto Sensor::transformation() -> Eigen::Map<Transformation> {
  return Eigen::Map<Transformation>{parameters_[kTransformationOffset]};
}

auto operator<<(YAML::Emitter& emitter, const Sensor& sensor) -> YAML::Emitter& {
  emitter << YAML::BeginMap;
  sensor.write(emitter);
  emitter << YAML::EndMap;
  return emitter;
}

Sensor::Sensor(const Index& num_variables, const Node& node) : rate_{kDefaultRate}, variables_{num_variables}, parameters_{num_variables} {
  // Initialize variables.
  DCHECK_LE(kNumVariables, variables_.size());
  DCHECK_LE(kNumVariables, parameters_.size());
  variables_[kOffsetOffset] = std::make_unique<Offset>();
  variables_[kTransformationOffset] = std::make_unique<Transformation>();
  parameters_[kOffsetOffset] = variables_[kOffsetOffset]->asVector().data();
  parameters_[kTransformationOffset] = variables_[kTransformationOffset]->asVector().data();

  // Read YAML node.
  if (!node.IsNull()) {
    read(node);
  }
}

auto Sensor::write(Emitter& emitter) const -> void {
  yaml::Write(emitter, kRateName, rate());
  yaml::WriteVariable(emitter, kOffsetName, offset());
  yaml::WriteVariable(emitter, kTransformationName, transformation());
}

auto Sensor::read(const Node& node) -> void {
  rate_ = yaml::ReadAs<Rate>(node, kRateName);
  offset() = yaml::ReadVariable<Offset>(node, kOffsetName);
  transformation() = yaml::ReadVariable<Transformation>(node, kTransformationName);
}

}  // namespace hyper::sensors
