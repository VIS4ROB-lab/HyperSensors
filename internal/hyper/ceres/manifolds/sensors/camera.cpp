/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/ceres/manifolds/sensors/camera.hpp"
#include "hyper/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/variables/intrinsics.hpp"

namespace hyper::ceres::manifolds {

using namespace sensors;

Manifold<Camera>::Manifold(const Camera* camera, const bool constant) : Manifold<Sensor>{camera, kNumSubmanifolds, constant} {
  setIntrinsicsConstant(constant);
  setDistortionConstant(constant);
}

auto Manifold<Camera>::sensor() const -> const Camera* {
  return static_cast<const Camera*>(sensor_);  // NOLINT
}

auto Manifold<Camera>::intrinsicsSubmanifold() const -> Submanifold* {
  return submanifolds_[Camera::kIntrinsicsOffset].get();
}

auto Manifold<Camera>::setIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), Camera::Intrinsics::kNumParameters);
  submanifolds_[Camera::kIntrinsicsOffset] = std::move(submanifold);
}

auto Manifold<Camera>::setIntrinsicsConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<Camera::Intrinsics>>(constant);
  setIntrinsicsSubmanifold(std::move(submanifold));
}

auto Manifold<Camera>::distortionSubmanifold() const -> Submanifold* {
  return submanifolds_[Camera::kDistortionOffset].get();
}

auto Manifold<Camera>::setDistortionSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), sensor()->distortion().asVector().size());
  submanifolds_[Camera::kDistortionOffset] = std::move(submanifold);
}

auto Manifold<Camera>::setDistortionConstant(const bool constant) -> void {
  const auto num_parameters = sensor()->distortion().asVector().size();
  auto submanifold = std::make_unique<Manifold<variables::Cartesian<Scalar, Eigen::Dynamic>>>(num_parameters, constant);
  setDistortionSubmanifold(std::move(submanifold));
}

}  // namespace hyper::ceres::manifolds
