/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/sensor.hpp"
#include "hyper/variables/bearing.hpp"
#include "hyper/variables/distortions/distortion.hpp"

namespace hyper::sensors {

class Camera final : public Sensor {
 public:
  // Constants.
  static constexpr auto kIntrinsicsOffset = Sensor::kNumVariables;
  static constexpr auto kDistortionOffset = kIntrinsicsOffset + 1;
  static constexpr auto kNumVariables = kDistortionOffset + 1;

  // Sensor size.
  struct SensorSize {
    Index width, height;
  };

  // Shutter type.
  enum class ShutterType { GLOBAL, VERTICAL, HORIZONTAL, DEFAULT = GLOBAL };

  // Shutter delta (i.e. increment between readouts).
  using ShutterDelta = Time;
  using Pixel = variables::Pixel<Scalar>;
  using Bearing = variables::Bearing<Scalar>;
  using Landmark = variables::Position<Scalar, 3>;
  using Intrinsics = variables::Intrinsics<Scalar>;
  using Distortion = variables::Distortion<Scalar>;

  /// Projects landmarks to the (normalized) image plane.
  /// \param landmark Landmark to project.
  /// \param J_l Jacobian of the projection.
  /// \return Projected pixel coordinates.
  static auto ProjectToPlane(const Eigen::Ref<const Landmark>& landmark, Scalar* J_l = nullptr) -> Pixel;

  /// Projects landmarks to the unit sphere.
  /// \param landmark Landmark to project.
  /// \param J_l Jacobian of the projection.
  /// \return Projected bearing coordinates.
  static auto ProjectToSphere(const Eigen::Ref<const Landmark>& landmark, Scalar* J_l = nullptr) -> Bearing;

  /// Lifts (normalized) pixel coordinates to the unit sphere.
  /// \param pixel Pixel to lift.
  /// \param J_p Jacobian of the lift.
  /// \return Bearing of lifted pixel coordinates.
  static auto LiftToSphere(const Eigen::Ref<const Pixel>& pixel, Scalar* J_p = nullptr) -> Bearing;

  /// Triangulates a landmark from relative a transformation and bearings.
  /// \param T_ab Input transformation.
  /// \param b_a Input bearing in frame a.
  /// \param b_b Input bearing in frame b.
  /// \return Triangulated landmark.
  static auto Triangulate(const Eigen::Ref<const Transformation>& T_ab, const Eigen::Ref<const Bearing>& b_a, const Eigen::Ref<const Bearing>& b_b) -> Landmark;

  /// Constructor from YAML file.
  /// \param node Input YAML node.
  explicit Camera(const Node& node = {});

  /// Sensor size accessor.
  /// \return Reference to sensor size.
  [[nodiscard]] auto sensorSize() const -> const SensorSize&;

  /// Sensor size modifier.
  /// \return Reference to sensor size.
  [[nodiscard]] auto sensorSize() -> SensorSize&;

  /// Shutter type accessor.
  /// \return Shutter type.
  [[nodiscard]] auto shutterType() const -> const ShutterType&;

  /// Shutter type modifier.
  /// \return Shutter type.
  [[nodiscard]] auto shutterType() -> ShutterType&;

  /// Shutter delta accessor.
  /// \return Shutter delta.
  [[nodiscard]] auto shutterDelta() const -> const ShutterDelta&;

  /// Shutter delta modifier.
  /// \return Shutter delta.
  [[nodiscard]] auto shutterDelta() -> ShutterDelta&;

  /// Intrinsics accessor.
  /// \return Reference to intrinsics.
  [[nodiscard]] auto intrinsics() const -> Eigen::Map<const Intrinsics>;

  /// Intrinsics modifier.
  /// \return Reference to intrinsics.
  auto intrinsics() -> Eigen::Map<Intrinsics>;

  /// Distortion accessor.
  /// \return Reference to distortion.
  [[nodiscard]] auto distortion() const -> const Distortion&;

  /// Distortion modifier.
  /// \return Reference to distortion.
  auto distortion() -> Distortion&;

  /// Sets the distortion.
  /// \tparam DistortionType Distortion type.
  /// \param distortion Distortion to be set.
  auto setDistortion(std::unique_ptr<Distortion>&& distortion) -> void;

  /// Corrects the shutter times.
  /// \param time Global shutter time.
  /// \param pixels Input pixels.
  /// \return Corrected shutter times.
  [[nodiscard]] auto correctShutterTimes(const Time& time, const std::vector<Pixel>& pixels) const -> std::vector<Time>;

  /// Converts pixels to bearings.
  /// \param pixels Input pixels.
  /// \param parameters External distortion parameters (optional).
  /// \return Bearings.
  [[nodiscard]] auto convertPixelsToBearings(const std::vector<Pixel>& pixels, const Scalar* parameters = nullptr) const -> std::vector<Bearing>;

  /// Triangulates a landmark from bearings.
  /// \param other Other camera.
  /// \param b_this Bearing in this frame.
  /// \param b_other Bearing in other frame.
  /// \return Triangulated landmark.
  auto triangulate(const Camera& other, const Eigen::Ref<const Bearing>& b_this, const Eigen::Ref<const Bearing>& b_other) -> Landmark;

 private:
  /// Reads the sensor size.
  /// \param node Input YAML node.
  /// \return Sensor size.
  static auto ReadSensorSize(const Node& node) -> SensorSize;

  /// Reads the shutter type.
  /// \param node Input YAML node.
  /// \return Shutter type.
  static auto ReadShutterType(const Node& node) -> ShutterType;

  /// Reads the shutter delta.
  /// \param node Input YAML node.
  /// \return Shutter delta.
  static auto ReadShutterDelta(const Node& node) -> ShutterDelta;

  /// Reads the distortion.
  /// \param node Input YAML node.
  /// \return Distortion
  static auto ReadDistortion(const Node& node) -> std::unique_ptr<Distortion>;

  /// Reads the sensor information from a YAML node.
  /// \param node Input YAML node.
  auto read(const Node& node) -> void;

  /// Writes the sensor size.
  /// \param emitter Modifiable emitter.
  auto writeSensorSize(Emitter& emitter) const -> void;

  /// Write the shutter type.
  /// \param emitter Modifiable emitter.
  auto writeShutterType(Emitter& emitter) const -> void;

  /// Writes the shutter delta.
  /// \param emitter Modifiable emitter.
  auto writeShutterDelta(Emitter& emitter) const -> void;

  /// Writes the distortion.
  /// \param emitter Modifiable emitter.
  auto writeDistortion(Emitter& emitter) const -> void;

  /// Writes the sensor information to a YAML emitter.
  /// \param emitter Output YAML emitter.
  auto write(Emitter& emitter) const -> void final;

  SensorSize sensor_size_;      ///< Sensor size.
  ShutterType shutter_type_;    ///< Shutter type.
  ShutterDelta shutter_delta_;  ///< Shutter delta.
};

}  // namespace hyper::sensors
