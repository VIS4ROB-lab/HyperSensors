/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/sensors/sensor.hpp"
#include "hyper/variables/groups/se3.hpp"

namespace hyper::sensors {

namespace {

// Variable names.
constexpr auto kRateName = "rate";
constexpr auto kTransformationName = "transformation";

// Default parameters.
constexpr auto kDefaultRate = -1;

}  // namespace

Sensor::Sensor(const Node& node) : Sensor{kNumParameters, node} {}

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

auto Sensor::transformation() const -> Eigen::Map<const Transformation> {
  const auto vector = variableAsVector(kTransformationOffset);
  return Eigen::Map<const Transformation>{vector.data()};
}

auto Sensor::transformation() -> Eigen::Map<Transformation> {
  auto vector = variableAsVector(kTransformationOffset);
  return Eigen::Map<Transformation>{vector.data()};
}

auto operator<<(YAML::Emitter& emitter, const Sensor& sensor) -> YAML::Emitter& {
  emitter << YAML::BeginMap;
  sensor.writeVariables(emitter);
  emitter << YAML::EndMap;
  return emitter;
}

Sensor::Sensor(const Index& num_variables, const Node& node) : rate_{kDefaultRate}, variables_{num_variables} {
  initializeVariables();
  if (!node.IsNull()) {
    readVariables(node);
  }
}

auto Sensor::variableAsVector(const Index& index) const -> Eigen::Ref<VectorX<Scalar>> {
  DCHECK_LT(index, variables_.size());
  DCHECK(variables_[index] != nullptr);
  return variables_[index]->asVector();
}

auto Sensor::writeVariables(Emitter& emitter) const -> void {
  yaml::Write(emitter, kRateName, rate());
  yaml::WriteVariable(emitter, kTransformationName, transformation());
}

auto Sensor::initializeVariables() -> void {
  DCHECK_LE(kNumParameters, variables_.size());
  variables_[kTransformationOffset] = std::make_unique<Transformation>();
}

auto Sensor::readVariables(const Node& node) -> void {
  rate_ = yaml::ReadAs<Rate>(node, kRateName);
  transformation() = yaml::ReadVariable<Transformation>(node, kTransformationName);
}

}  // namespace hyper::sensors
