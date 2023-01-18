/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <cv_bridge/cv_bridge.h>

#include "hyper/messages/abstract.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/variables/cartesian.hpp"

namespace hyper {

class VisualTracks
    : public AbstractMessage {
 public:
  // Definitions.
  using Image = cv_bridge::CvImageConstPtr;

  using Camera = sensors::Camera;

  using Points = std::vector<cv::Point2f>;
  using Entry = std::tuple<Image, Points>;
  using Tracks = std::map<const Camera*, Entry>;
  using Identifiers = std::vector<Identifier>;
  using Positions = std::vector<Position<Scalar>>;
  using Lengths = std::vector<std::size_t>;

  /// Constructor from time and sensor.
  /// \param stamp Stamp.
  /// \param camera Camera.
  VisualTracks(const Stamp& stamp, const Camera& camera);

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const Camera& final;

  /// Sets the associated sensor.
  /// \param camera Sensor to set.
  auto setSensor(const Camera& camera) -> void;

  /// Adds a new track.
  /// \param camera Associated camera.
  auto addTrack(const Camera& camera) -> Entry&;

  /// Track accessor.
  /// \param camera Associated camera.
  /// \return Entry containing the associated image and tracked points.
  [[nodiscard]] auto getTrack(const Camera& camera) const -> const Entry&;

  /// Track modifier.
  /// \param camera Associated camera.
  /// \return Entry containing the associated image and tracked points.
  auto getTrack(const Camera& camera) -> Entry&;

  Tracks tracks;           ///< Tracks.
  Identifiers identifiers; ///< Track identifiers.
  Positions positions;     ///< Track positions (optional).
  Lengths lengths;         ///< Track lengths.
};

class StereoVisualTracks
    : public VisualTracks {
  /// Constructor from time and sensor.
  /// \param stamp Stamp.
  /// \param camera Camera.
  /// \param other_camera Other camera.
  StereoVisualTracks(const Stamp& stamp, const Camera& camera, const Camera& other_camera);

  /// Other sensor accessor.
  /// \return Other sensor.
  [[nodiscard]] auto otherSensor() const -> const Camera&;

  /// Sets the associated other sensor.
  /// \param other_camera Other sensor to set.
  auto setOtherSensor(const Camera& other_camera) -> void;

 private:
  const Sensor* other_sensor_; ///< Other sensor.
};

} // namespace hyper
