/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/measurement.hpp"
#include "hyper/sensors/imu.hpp"

namespace hyper::messages {

template <typename TManifold>
class InertialMeasurement final
    : public MeasurementBase<variables::Tangent<TManifold>> {
 public:
  // Definitions.
  using Manifold = TManifold;
  using Tangent = variables::Tangent<TManifold>;
  using Base = MeasurementBase<variables::Tangent<TManifold>>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  using IMU = sensors::IMU;

  /// Constructor from time, sensor and value.
  /// \param time Time.
  /// \param imu Sensor.
  /// \param tangent Tangent.
  InertialMeasurement(const Time& time, const IMU& imu, const Tangent& tangent)
      : Base{time, tangent}, imu_{&imu} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] inline auto sensor() const -> const IMU& final { *imu_; }

  /// Sets the associated sensor.
  /// \param imu Sensor to set.
  inline auto setSensor(const IMU& imu) -> void { imu_ = &imu; }

 private:
  const IMU* imu_;  ///< IMU.
};

}  // namespace hyper::messages
