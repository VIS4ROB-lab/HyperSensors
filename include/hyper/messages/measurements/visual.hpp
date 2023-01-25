/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/variable.hpp"
#include "hyper/sensors/camera.hpp"

namespace hyper::messages {

template <typename TVariable>
class CameraMeasurement final : public VariableMeasurement<TVariable> {
 public:
  /// Constructor from time, sensor and value.
  /// \param stamp Stamp.
  /// \param camera Camera.
  /// \param variable Variable.
  CameraMeasurement(const Stamp& stamp, const Camera& camera, const TVariable& variable) : VariableMeasurement<TVariable>{stamp, camera, variable} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const Camera& final {
    return static_cast<const Camera&>(*this->sensor_);  // NOLINT
  }

  /// Sets the associated sensor.
  /// \param camera Sensor to set.
  auto setSensor(const Camera& camera) -> void { this->sensor_ = &camera; }
};

using PixelMeasurement = CameraMeasurement<Pixel<Scalar>>;

using BearingMeasurement = CameraMeasurement<Bearing<Scalar>>;

}  // namespace hyper::messages
