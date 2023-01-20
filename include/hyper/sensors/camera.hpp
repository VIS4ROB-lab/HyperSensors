/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/sensor.hpp"
#include "hyper/variables/bearing.hpp"
#include "hyper/variables/distortions/distortion.hpp"
#include "hyper/variables/intrinsics.hpp"

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

  /// Converts a landmark to a (normalized) pixel coordinate.
  /// \param landmark Landmark.
  /// \param J_l Jacobian (optional).
  /// \return Pixel.
  static auto LandmarkToPixel(const Eigen::Ref<const Landmark>& landmark, Scalar* J_l = nullptr) -> Pixel;

  /// Converts a landmark to a bearing.
  /// \param landmark Landmark.
  /// \param J_l Jacobian (optional).
  /// \return Bearing.
  static auto LandmarkToBearing(const Eigen::Ref<const Landmark>& landmark, Scalar* J_l = nullptr) -> Bearing;

  /// Converts a (normalized) pixel coordinate to a bearing.
  /// \param pixel Pixel.
  /// \param J_p Jacobian (optional).
  /// \return Bearing.
  static auto PixelToBearing(const Eigen::Ref<const Pixel>& pixel, Scalar* J_p = nullptr) -> Bearing;

  /// Triangulates a landmark from relative a transformation and bearings.
  /// \param T_ab Input transformation.
  /// \param b_a Input bearing in frame a.
  /// \param b_b Input bearing in frame b.
  /// \return Triangulated landmark.
  static auto Triangulate(const Eigen::Ref<const Transformation>& T_ab, const Eigen::Ref<const Bearing>& b_a, const Eigen::Ref<const Bearing>& b_b) -> Landmark;

  /// Reads the distortion.
  /// \param node Node
  /// \return Distortion
  static auto ReadDistortion(const Node& node) -> std::unique_ptr<Distortion>;

  /// Default constructor.
  explicit Camera(std::unique_ptr<Distortion>&& distortion);

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
  /// \param node Node
  /// \return Sensor size.
  static auto ReadSensorSize(const Node& node) -> SensorSize;

  /// Reads the shutter type.
  /// \param node Node
  /// \return Shutter type.
  static auto ReadShutterType(const Node& node) -> ShutterType;

  /// Reads the shutter delta.
  /// \param node Node
  /// \return Shutter delta.
  static auto ReadShutterDelta(const Node& node) -> ShutterDelta;

  /// Reads a sensor from a YAML node.
  /// \param node YAML node.
  auto read(const Node& node) -> void final;

  /// Writes the sensor size.
  /// \param emitter Emitter.
  auto writeSensorSize(Emitter& emitter) const -> void;

  /// Write the shutter type.
  /// \param emitter Emitter.
  auto writeShutterType(Emitter& emitter) const -> void;

  /// Writes the shutter delta.
  /// \param emitter Emitter.
  auto writeShutterDelta(Emitter& emitter) const -> void;

  /// Writes the distortion.
  /// \param emitter Emitter.
  auto writeDistortion(Emitter& emitter) const -> void;

  /// Writes a sensor to a YAML emitter.
  /// \param emitter YAML emitter.
  auto write(Emitter& emitter) const -> void final;

  SensorSize sensor_size_;      ///< Sensor size.
  ShutterType shutter_type_;    ///< Shutter type.
  ShutterDelta shutter_delta_;  ///< Shutter delta.
};

}  // namespace hyper::sensors
