/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#ifdef HYPER_COMPILE_WITH_CERES

#include <ceres/manifold.h>

#include "hyper/ceres/manifolds/sensors/forward.hpp"

#include "hyper/sensors/sensor.hpp"

namespace hyper::ceres::manifolds {

template <>
class Manifold<sensors::Sensor> {
 public:
  // Definitions.
  using Time = double;
  using Scalar = double;
  using Sensor = sensors::Sensor;
  using Submanifold = ::ceres::Manifold;

  // Constants.
  static constexpr auto kNumSubmanifolds = Sensor::kNumVariables;

  /// Constructor from sensor and constancy flag.
  /// \param sensor Input sensor.
  /// \param constant Constancy flag.
  explicit Manifold(const Sensor* sensor, bool constant = true);

  /// Default destructor.
  virtual ~Manifold();

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] virtual auto sensor() const -> const Sensor*;

  /// Offset submanifold accessor.
  /// \return Offset submanifold.
  [[nodiscard]] auto offsetSubmanifold() const -> Submanifold*;

  /// Sets the offset submanifold.
  /// \param submanifold Input submanifold.
  auto setOffsetSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the offset submanifold constant.
  /// \param constant Constancy flag.
  auto setOffsetConstant(bool constant) -> void;

  /// Transformation submanifold accessor.
  /// \return Transformation submanifold.
  [[nodiscard]] auto transformationSubmanifold() const -> Submanifold*;

  /// Sets the transformation submanifold.
  /// \param submanifold Input submanifold.
  auto setTransformationSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void;

  /// Sets the transformation submanifold constant.
  /// \param constant Constancy flag.
  auto setTransformationConstant(bool constant) -> void;

  /// Retrieves all submanifolds.
  /// \return Submanifolds.
  [[nodiscard]] virtual auto submanifolds() const -> std::vector<Submanifold*>;

  /// Retrieves all submanifolds (time-based).
  /// \param time Query time.
  /// \return Submanifolds.
  [[nodiscard]] virtual auto submanifolds(const Time& time) const -> std::vector<Submanifold*>;

 protected:
  // Definitions.
  using Size = std::size_t;
  using Submanifolds = std::vector<std::unique_ptr<Submanifold>>;

  /// Constructor from sensor, number of submanifolds and constancy flag.
  /// \param sensor Input sensor.
  /// \param num_submanifolds Number of submanifolds.
  /// \param constant Constancy flag.
  Manifold(const Sensor* sensor, const Size& num_submanifolds, bool constant);

  const Sensor* sensor_;       ///< Sensor.
  Submanifolds submanifolds_;  ///< Submanifolds.
};

}  // namespace hyper::ceres::manifolds

#endif
