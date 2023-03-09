/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/measurement.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/variables/cartesian.hpp"

namespace hyper::messages {

template <typename TValue>
class VisualMeasurement final : public MeasurementBase<TValue> {
 public:
  // Definitions.
  using Base = MeasurementBase<TValue>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  using Camera = sensors::Camera;

  /// Constructor from time, camera and value.
  /// \param time Time.
  /// \param camera Camera.
  /// \param value Value.
  VisualMeasurement(const Time& time, const Camera* camera, const TValue& value) : Base{Type::VISUAL_MEASUREMENT, time, value}, camera_{camera} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const Camera* final { return camera_; }

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
