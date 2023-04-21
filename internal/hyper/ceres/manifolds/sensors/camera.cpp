/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#ifdef HYPER_COMPILE_WITH_CERES

#include "hyper/ceres/manifolds/sensors/camera.hpp"
#include "hyper/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/variables/intrinsics.hpp"

namespace hyper::ceres::manifolds {

using namespace sensors;

Manifold<Camera>::Manifold(Camera* camera, const bool constant) : Manifold<Sensor>{camera, kNumSubmanifolds, constant} {
  setIntrinsicsConstant(constant);
  setDistortionConstant(constant);
}

auto Manifold<Camera>::sensor() const -> const Camera* {
  return static_cast<const Camera*>(sensor_);  // NOLINT
}

auto Manifold<Camera>::intrinsicsSubmanifold() const -> Submanifold* {
  return submanifolds_[Camera::kIntrinsicsIndex].get();
}

auto Manifold<Camera>::setIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), Camera::Intrinsics::kNumParameters);
  submanifolds_[Camera::kIntrinsicsIndex] = std::move(submanifold);
}

auto Manifold<Camera>::setIntrinsicsConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<Camera::Intrinsics>>(constant);
  setIntrinsicsSubmanifold(std::move(submanifold));
}

auto Manifold<Camera>::distortionSubmanifold() const -> Submanifold* {
  return submanifolds_[Camera::kDistortionIndex].get();
}

auto Manifold<Camera>::setDistortionSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  if (!sensor()->distortion()) {
    CHECK(submanifold == nullptr) << "Submanifold of distortion-free camera must be null.";
    submanifolds_[Camera::kDistortionIndex] = nullptr;
    return;
  }

  DCHECK_EQ(submanifold->AmbientSize(), sensor()->distortion()->asVector().size());
  submanifolds_[Camera::kDistortionIndex] = std::move(submanifold);
}

auto Manifold<Camera>::setDistortionConstant(const bool constant) -> void {
  if (!sensor()->distortion()) {
    setDistortionSubmanifold(nullptr);
    return;
  }

  const auto num_parameters = sensor()->distortion()->asVector().size();
  auto submanifold = std::make_unique<Manifold<variables::Rn<Scalar, Eigen::Dynamic>>>(num_parameters, constant);
  setDistortionSubmanifold(std::move(submanifold));
}

}  // namespace hyper::ceres::manifolds

#endif
