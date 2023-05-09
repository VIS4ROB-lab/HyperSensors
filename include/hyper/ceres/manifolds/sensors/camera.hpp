/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#if HYPER_COMPILE_WITH_CERES

#include "hyper/ceres/manifolds/sensors/sensor.hpp"
#include "hyper/sensors/camera.hpp"

namespace hyper::ceres::manifolds {

template <>
class Manifold<sensors::Camera> final : public Manifold<sensors::Sensor> {
 public:
  // Definitions.
  using Camera = sensors::Camera;

  // Constants.
  static constexpr auto kNumSubmanifolds = Camera::kNumVariables;

  /// Default constructor.
  /// \param camera Camera to parametrize.
  /// \param constant Constancy flag.
  explicit Manifold(Camera* camera, bool constant = true);

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const Camera* final;

  /// Intrinsics submanifold accessor.
  [[nodiscard]] auto intrinsicsSubmanifold() const -> Submanifold*;

  /// Sets the intrinsics submanifold.
  /// \param submanifold Input submanifold.
  auto setIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the intrinsics constant.
  /// \param constant Constancy flag.
  auto setIntrinsicsConstant(bool constant) -> void;

  /// Distortion submanifold accessor.
  [[nodiscard]] auto distortionSubmanifold() const -> Submanifold*;

  /// Sets the distortion submanifold.
  /// \param submanifold Input submanifold.
  auto setDistortionSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the distortion constant.
  /// \param constant Constancy flag.
  auto setDistortionConstant(bool constant) -> void;
};

}  // namespace hyper::ceres::manifolds

#endif
