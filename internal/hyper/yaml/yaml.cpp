/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/yaml/yaml.hpp"

namespace hyper::yaml {

auto Read(const Node& node, const Key& key) -> Node {
  const auto value = node[key];
  CHECK(value) << "'" << key << "' does not exist.";
  return value;
}

auto ReadString(const Node& node, const Key& key) -> String {
  auto string = ReadAs<String>(node, key);
  std::transform(string.begin(), string.end(), string.begin(), ::tolower);
  return string;
}

} // namespace hyper::yaml
