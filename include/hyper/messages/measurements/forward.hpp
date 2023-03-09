/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/forward.hpp"

namespace hyper::messages {

template <typename TScalar>
class Measurement;

template <typename TValue>
class AbsoluteMeasurement;

template <typename TValue>
class VisualMeasurement;

template <typename TManifold>
class InertialMeasurement;

template <typename TSensor, typename TValue>
class RelativeMeasurement;

}  // namespace hyper::messages
