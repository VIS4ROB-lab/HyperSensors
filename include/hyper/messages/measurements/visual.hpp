/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/variable.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/variables/cartesian.hpp"

namespace hyper::messages {

template <typename TVariable>
class CameraMeasurement final : public VariableMeasurement<TVariable> {
 public:
  // Definitions.
  using Variable = TVariable;
  using Base = VariableMeasurement<TVariable>;
  using Time = typename Base::Time;

  using Camera = sensors::Camera;

  /// Constructor from time, camera and variable.
  /// \param time Time.
  /// \param camera Camera.
  /// \param variable Variable.
  CameraMeasurement(const Time& time, const Camera& camera,
                    const TVariable& variable)
      : VariableMeasurement<TVariable>{time, variable}, camera_{&camera} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const Camera& final { *camera_; }

  /// Sets the associated sensor.
  /// \param camera Sensor to set.
  inline auto setSensor(const Camera& camera) -> void { camera_ = &camera; }

 private:
  const Camera* camera_;  ///< Camera.
};

template <typename TScalar>
using PixelMeasurement = CameraMeasurement<variables::Pixel<TScalar>>;

template <typename TScalar>
using BearingMeasurement = CameraMeasurement<variables::Bearing<TScalar>>;

}  // namespace hyper::messages
