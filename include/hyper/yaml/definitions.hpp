/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <yaml-cpp/yaml.h>

namespace hyper::yaml {

using Key = std::string;
using String = std::string;

using Node = YAML::Node;
using Emitter = YAML::Emitter;

}  // namespace hyper::yaml
