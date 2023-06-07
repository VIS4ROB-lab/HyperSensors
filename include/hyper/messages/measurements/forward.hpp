/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/forward.hpp"
#include "hyper/variables/forward.hpp"

namespace hyper::messages {

class Measurement;

template <typename TValue>
class AbsoluteMeasurement;

template <typename TValue>
class VisualMeasurement;

template <typename TManifold>
class InertialMeasurement;

template <typename TSensor, typename TValue>
class RelativeMeasurement;

template <typename TManifold>
using ManifoldMeasurement = AbsoluteMeasurement<TManifold>;

template <typename TManifold>
using TangentMeasurement = AbsoluteMeasurement<variables::Tangent<TManifold>>;

using PixelMeasurement = VisualMeasurement<variables::R2>;

using BearingMeasurement = VisualMeasurement<variables::Bearing>;

template <typename TSensor, typename TManifold>
using RelativeManifoldMeasurement = RelativeMeasurement<TSensor, TManifold>;

template <typename TSensor, typename TManifold>
using RelativeTangentMeasurement = RelativeMeasurement<TSensor, variables::Tangent<TManifold>>;

}  // namespace hyper::messages
