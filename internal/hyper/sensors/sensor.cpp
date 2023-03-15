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

Sensor::Sensor(JacobianType jacobian_type) : Sensor{Type::ABSOLUTE, jacobian_type, kNumVariables} {}

Sensor::Sensor(const Node& node) : Sensor{} {
  node >> *this;
}

auto Sensor::type() const -> Type {
  return type_;
}

auto Sensor::jacobianType() const -> JacobianType {
  return jacobian_type_;
}

auto Sensor::setJacobianType(JacobianType jacobian_type) -> void {
  jacobian_type_ = jacobian_type;
  updateParameterBlockSizes();
}

auto Sensor::rate() const -> const Rate& {
  return rate_;
}

auto Sensor::hasVariableRate() const -> bool {
  return rate() < Scalar{0};
}

auto Sensor::partitions(const Time& /* time */) const -> Partitions<Scalar*> {
  Partitions<Scalar*> partitions{kNumPartitions};
  auto& [v_offset, v_parameter_blocks, v_parameter_block_sizes] = partitions[kVariablesPartitionIndex];
  v_offset = kVariablesOffset;
  v_parameter_blocks = parameter_blocks_;
  v_parameter_block_sizes = parameter_block_sizes_;
  return partitions;
}

auto Sensor::offset() const -> const Offset& {
  return static_cast<const Offset&>(*variables_[kOffsetIndex]);  // NOLINT
}

auto Sensor::offset() -> Offset& {
  return const_cast<Offset&>(std::as_const(*this).offset());
}

auto Sensor::transformation() const -> const Transformation& {
  return static_cast<const Transformation&>(*variables_[kTransformationIndex]);  // NOLINT
}

auto Sensor::transformation() -> Transformation& {
  return const_cast<Transformation&>(std::as_const(*this).transformation());
}

auto operator>>(const YAML::Node& node, Sensor& sensor) -> const YAML::Node& {
  CHECK(node);
  sensor.read(node);
  return node;
}

auto operator<<(YAML::Emitter& emitter, const Sensor& sensor) -> YAML::Emitter& {
  emitter << YAML::BeginMap;
  sensor.write(emitter);
  emitter << YAML::EndMap;
  return emitter;
}

Sensor::Sensor(const Type type, JacobianType jacobian_type, Size num_variables)
    : type_{type}, jacobian_type_{jacobian_type}, rate_{kDefaultRate}, variables_(num_variables), parameter_blocks_(num_variables), parameter_block_sizes_(num_variables) {
  // Initialize variables.
  DCHECK_LE(kNumVariables, variables_.size());
  DCHECK_LE(kNumVariables, parameter_blocks_.size());
  DCHECK_LE(kNumVariables, parameter_block_sizes_.size());
  variables_[kOffsetIndex] = std::make_unique<Offset>();
  variables_[kTransformationIndex] = std::make_unique<Transformation>();
  parameter_blocks_[kOffsetIndex] = variables_[kOffsetIndex]->asVector().data();
  parameter_blocks_[kTransformationIndex] = variables_[kTransformationIndex]->asVector().data();
  updateSensorParameterBlockSizes();
}

auto Sensor::updateSensorParameterBlockSizes() -> void {
  if (jacobian_type_ == JacobianType::TANGENT_TO_MANIFOLD) {
    parameter_block_sizes_[kOffsetIndex] = Offset::kNumParameters;
    parameter_block_sizes_[kTransformationIndex] = Transformation::kNumParameters;
  } else {
    parameter_block_sizes_[kOffsetIndex] = variables::Tangent<Offset>::kNumParameters;
    parameter_block_sizes_[kTransformationIndex] = variables::Tangent<Transformation>::kNumParameters;
  }
}

auto Sensor::updateParameterBlockSizes() -> void {
  updateSensorParameterBlockSizes();
}

auto Sensor::read(const Node& node) -> void {
  rate_ = yaml::ReadAs<Rate>(node, kRateName);
  offset() = yaml::ReadVariable<Offset>(node, kOffsetName);
  transformation() = yaml::ReadVariable<Transformation>(node, kTransformationName);
}

auto Sensor::write(Emitter& emitter) const -> void {
  yaml::Write(emitter, kRateName, rate());
  yaml::WriteVariable(emitter, kOffsetName, offset());
  yaml::WriteVariable(emitter, kTransformationName, transformation());
}

}  // namespace hyper::sensors
