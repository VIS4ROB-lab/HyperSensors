/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/forward.hpp"

namespace hyper::messages {

template <typename TScalar>
class Measurement;

template <typename TVariable>
class AbsoluteMeasurement;

template <typename TVariable>
class VisualMeasurement;

template <typename TManifold>
class InertialMeasurement;

template <typename TSensor, typename TVariable>
class RelativeMeasurement;

}  // namespace hyper::messages
