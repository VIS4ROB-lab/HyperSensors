/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/sensor.hpp"
#include "hyper/variables/bearing.hpp"
#include "hyper/variables/distortions/abstract.hpp"

namespace hyper {

class Camera final : public Sensor {
 public:
  // Definitions.
  using SensorSize = Traits<Camera>::SensorSize;
  using ShutterType = Traits<Camera>::ShutterType;
  using ShutterDelta = Traits<Camera>::ShutterDelta;

  /// Projects positions to the (normalized) image plane.
  /// \param position Position to project.
  /// \param raw_J Jacobian of the projection.
  /// \return Projected pixel coordinates.
  static auto ProjectToPlane(const Eigen::Ref<const Position<Scalar>>& position, Scalar* raw_J = nullptr) -> Pixel<Scalar>;

  /// Projects positions to the unit sphere.
  /// \param position Position to project.
  /// \param raw_J Jacobian of the projection.
  /// \return Projected bearing coordinates.
  static auto ProjectToSphere(const Eigen::Ref<const Position<Scalar>>& position, Scalar* raw_J = nullptr) -> Bearing<Scalar>;

  /// Lifts (normalized) pixel coordinates to the unit sphere.
  /// \param pixel Pixel to lift.
  /// \param raw_J Jacobian of the lift.
  /// \return Bearing of lifted pixel coordinates.
  static auto LiftToSphere(const Eigen::Ref<const Pixel<Scalar>>& pixel, Scalar* raw_J = nullptr) -> Bearing<Scalar>;

  /// Triangulates a position from relative a transformation and bearings.
  /// \param T_ab Input transformation.
  /// \param b_a Input bearing in frame a.
  /// \param b_b Input bearing in frame b.
  /// \return Triangulated position.
  static auto Triangulate(const Eigen::Ref<const Transformation>& T_ab, const Eigen::Ref<const Bearing<Scalar>>& b_a, const Eigen::Ref<const Bearing<Scalar>>& b_b) -> Position<Scalar>;

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
  [[nodiscard]] auto intrinsics() const -> Eigen::Map<const Intrinsics<Scalar>>;

  /// Intrinsics modifier.
  /// \return Reference to intrinsics.
  auto intrinsics() -> Eigen::Map<Intrinsics<Scalar>>;

  /// Distortion accessor.
  /// \return Reference to distortion.
  [[nodiscard]] auto distortion() const -> const AbstractDistortion<Scalar>&;

  /// Distortion modifier.
  /// \return Reference to distortion.
  auto distortion() -> AbstractDistortion<Scalar>&;

  /// Sets the distortion.
  /// \tparam DistortionType Distortion type.
  /// \param distortion Distortion to be set.
  auto setDistortion(std::unique_ptr<AbstractDistortion<Scalar>>&& distortion) -> void;

  /// Corrects the shutter stamps.
  /// \param stamp Global shutter stamp.
  /// \param pixels Input pixels.
  /// \return Corrected shutter stamps.
  [[nodiscard]] auto correctShutterStamps(const Stamp& stamp, const std::vector<Pixel<Scalar>>& pixels) const -> Stamps;

  /// Converts pixels to bearings.
  /// \param pixels Input pixels.
  /// \return Bearings.
  [[nodiscard]] auto convertPixelsToBearings(const std::vector<Pixel<Scalar>>& pixels) const -> std::vector<Bearing<Scalar>>;

  /// Triangulates a position from bearings.
  /// \param other Other camera.
  /// \param b_this Bearing in this frame.
  /// \param b_other Bearing in other frame.
  /// \return Triangulated position.
  auto triangulate(const Camera& other, const Eigen::Ref<const Bearing<Scalar>>& b_this, const Eigen::Ref<const Bearing<Scalar>>& b_other) -> Position<Scalar>;

 private:
  /// Initializes the parameters.
  auto initializeParameters() -> void;

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
  static auto ReadDistortion(const Node& node) -> std::unique_ptr<AbstractDistortion<Scalar>>;

  /// Reads all sensor parameters from a YAML node.
  /// \param node Input YAML node.
  auto readParameters(const Node& node) -> void;

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

  /// Outputs all sensor parameters to a YAML emitter.
  /// \param emitter Output YAML emitter.
  auto writeParameters(Emitter& emitter) const -> void final;

  SensorSize sensor_size_;     ///< Sensor size.
  ShutterType shutter_type_;   ///< Shutter type.
  ShutterDelta shutter_delta_; ///< Shutter delta.
};

} // namespace hyper
