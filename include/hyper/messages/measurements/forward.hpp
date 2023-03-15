/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/forward.hpp"
#include "hyper/variables/forward.hpp"

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

template <typename TManifold>
using ManifoldMeasurement = AbsoluteMeasurement<TManifold>;

template <typename TManifold>
using TangentMeasurement = AbsoluteMeasurement<variables::Tangent<TManifold>>;

template <typename TScalar>
using PixelMeasurement = VisualMeasurement<variables::Pixel<TScalar>>;

template <typename TScalar>
using BearingMeasurement = VisualMeasurement<variables::Bearing<TScalar>>;

template <typename TSensor, typename TManifold>
using RelativeManifoldMeasurement = RelativeMeasurement<TSensor, TManifold>;

template <typename TSensor, typename TManifold>
using RelativeTangentMeasurement = RelativeMeasurement<TSensor, variables::Tangent<TManifold>>;

}  // namespace hyper::messages
