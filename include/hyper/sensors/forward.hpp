/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/forward.hpp"

#include "hyper/definitions.hpp"
#include "hyper/variables/groups/se3.hpp"

namespace hyper {

template <typename>
struct Traits;

class Sensor;

template <>
struct Traits<Sensor> {
  // Definitions.
  using Rate = Scalar;
  using Transformation = SE3<Scalar>;

  // Constants.
  static constexpr auto kTransformationOffset = 0;
  static constexpr auto kNumParameters = kTransformationOffset + 1;
};

class Camera;

template <>
struct Traits<Camera> final
    : public Traits<Sensor> {
  // Sensor size in pixel.
  struct SensorSize {
    using Value = int;
    Value width, height;
  };

  // Shutter type.
  enum class ShutterType {
    GLOBAL,
    VERTICAL,
    HORIZONTAL,
    DEFAULT = GLOBAL
  };

  // Shutter delta (i.e. time increment
  // between line readouts).
  using ShutterDelta = Scalar;

  // Constants.
  static constexpr auto kIntrinsicsOffset = Traits<Sensor>::kNumParameters;
  static constexpr auto kDistortionOffset = kIntrinsicsOffset + 1;
  static constexpr auto kNumParameters = kDistortionOffset + 1;
};

class IMU;

template <>
struct Traits<IMU> final
    : public Traits<Sensor> {
  // Definitions.
  using GyroscopeNoiseDensity = Scalar;
  using GyroscopeIntrinsics = OrthonormalityAlignment<Scalar, 3>;
  using GyroscopeSensitivity = Cartesian<Scalar, 9>;
  using GyroscopeBias = Stamped<Cartesian<Scalar, 3>>;

  using AccelerometerNoiseDensity = Scalar;
  using AccelerometerIntrinsics = OrthonormalityAlignment<Scalar, 3>;
  using AccelerometerAxesOffsets = Cartesian<Scalar, 9>;
  using AccelerometerBias = Stamped<Cartesian<Scalar, 3>>;

  // Constants.
  static constexpr auto kGyroscopeIntrinsicsOffset = Traits<Sensor>::kNumParameters;
  static constexpr auto kAccelerometerIntrinsicsOffset = kGyroscopeIntrinsicsOffset + 1;
  static constexpr auto kAccelerometerAxesOffsetsOffset = kAccelerometerIntrinsicsOffset + 1;
  static constexpr auto kGyroscopeSensitivityOffset = kAccelerometerAxesOffsetsOffset + 1;
  static constexpr auto kNumParameters = kGyroscopeSensitivityOffset + 1;
};

} // namespace hyper
