/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/measurement.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/variables/cartesian.hpp"

namespace hyper::messages {

template <typename TVariable>
class VisualMeasurement final : public MeasurementBase<TVariable> {
 public:
  // Definitions.
  using Base = MeasurementBase<TVariable>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  using Camera = sensors::Camera;

  /// Constructor from time, camera and variable.
  /// \param time Time.
  /// \param camera Camera.
  /// \param variable Variable.
  VisualMeasurement(const Time& time, const Camera* camera,
                    const TVariable& variable)
      : Base{Type::VISUAL_MEASUREMENT, time, variable}, camera_{camera} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const Camera* final {
    return camera_;
  }

  /// Sets the associated sensor.
  /// \param camera Sensor to set.
  inline auto setSensor(const Camera* camera) -> void { camera_ = camera; }

 private:
  const Camera* camera_;  ///< Camera.
};

template <typename TScalar>
using PixelMeasurement = VisualMeasurement<variables::Pixel<TScalar>>;

template <typename TScalar>
using BearingMeasurement = VisualMeasurement<variables::Bearing<TScalar>>;

}  // namespace hyper::messages
