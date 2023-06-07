/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <glog/logging.h>

#include "hyper/variables/variable.hpp"
#include "hyper/yaml/definitions.hpp"

namespace hyper::yaml {

/// Safely reads a value from a YAML node with checks.
/// \param node YAML node to read from.
/// \param key Key of the value to read.
/// \return Read value node.
auto Read(const Node& node, const Key& key) -> Node;

/// Safely reads a value as type from a YAML node with checks.
/// \tparam TValue Type of value to be read.
/// \param node YAML node to read from.
/// \param key Key of the value to read.
/// \return Read value node.
template <typename TValue>
auto ReadAs(const Node& node, const Key& key) -> TValue {
  const auto value_node = Read(node, key);
  return value_node.template as<TValue>();
}

/// Safely reads a value as string from a YAML node with checks.
/// \param node YAML node to read from.
/// \param key Key of the value to read.
/// \return Read string (converted to lower case).
auto ReadString(const Node& node, const Key& key) -> String;

/// Safely reads parameters and constructs a target type instance.
/// \tparam TVariable Target type to construct.
/// \param node YAML node to read from.
/// \param key Key of the value to read.
/// \return Read parameters as target type.
template <typename TVariable>
auto ReadVariable(const Node& node, const Key& key) -> TVariable {
  const auto values = ReadAs<std::vector<Scalar>>(node, key);
  CHECK_EQ(values.size(), TVariable::kNumParameters);
  return TVariable{values.data()};
}

/// Writes out a value to a YAML emitter.
/// \tparam TValue Type of the value to be written.
/// \param emitter YAML emitter to write from.
/// \param key Key of the value to write out.
/// \param value value to be written.
/// \return Modified YAML emitter.
template <typename TValue>
auto Write(Emitter& emitter, const Key& key, const TValue& value) -> Emitter& {
  return emitter << YAML::Key << key << YAML::Value << value;
}

/// Writes out parameters to a YAML emitter.
/// \tparam TValue Type of the value to be written.
/// \param emitter YAML emitter to write from.
/// \param key Key of the value to write out.
/// \param memory_block Input memory block to write out.
/// \return Modified YAML emitter.
template <typename TVariable>
auto WriteVariable(Emitter& emitter, const Key& key, const TVariable& variable) -> Emitter& {
  const auto vector = variable.asVector();
  const auto data = vector.data();
  const auto size = vector.size();
  return emitter << YAML::Key << key << YAML::Value << YAML::Flow << std::vector<Scalar>{data, data + size};
}

}  // namespace hyper::yaml
