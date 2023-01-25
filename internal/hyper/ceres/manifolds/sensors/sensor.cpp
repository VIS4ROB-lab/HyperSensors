/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#ifdef HYPER_COMPILE_WITH_CERES

#include "hyper/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/ceres/manifolds/variables/groups/se3.hpp"

namespace hyper::ceres::manifolds {

using namespace sensors;

Manifold<Sensor>::Manifold(const Sensor* sensor, const bool constant) : Manifold{sensor, kNumSubmanifolds, constant} {}

Manifold<Sensor>::~Manifold() = default;

auto Manifold<Sensor>::sensor() const -> const Sensor* {
  return sensor_;
}

auto Manifold<Sensor>::offsetSubmanifold() const -> Submanifold* {
  return submanifolds_[Sensor::kOffsetIndex].get();
}

auto Manifold<Sensor>::setOffsetSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), Sensor::Offset::kNumParameters);
  submanifolds_[Sensor::kOffsetIndex] = std::move(submanifold);
}

auto Manifold<Sensor>::setOffsetConstant(bool constant) -> void {
  submanifolds_[Sensor::kOffsetIndex] = std::make_unique<Manifold<Sensor::Offset>>(constant);
}

auto Manifold<Sensor>::transformationSubmanifold() const -> Submanifold* {
  return submanifolds_[Sensor::kTransformationIndex].get();
}

auto Manifold<Sensor>::setTransformationSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), Sensor::Transformation::kNumParameters);
  submanifolds_[Sensor::kTransformationIndex] = std::move(submanifold);
}

auto Manifold<Sensor>::setTransformationConstant(bool constant) -> void {
  submanifolds_[Sensor::kTransformationIndex] = std::make_unique<Manifold<Sensor::Transformation>>(constant, constant);
}

Manifold<Sensor>::Manifold(const Sensor* sensor, const Size& num_submanifolds, const bool constant) : sensor_{sensor}, submanifolds_{num_submanifolds} {
  DCHECK(sensor != nullptr);
  setOffsetConstant(constant);
  setTransformationConstant(constant);
}

}  // namespace hyper::ceres::manifolds

#endif
