/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

#include "hyper/messages/message.hpp"
#include "hyper/sensors/camera.hpp"
#include "hyper/variables/cartesian.hpp"

namespace hyper::messages {

template <typename TScalar>
class VisualTracks : public Message<TScalar> {
 public:
  // Definitions.
  using Base = Message<TScalar>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  using Index = Eigen::Index;
  using ID = Index;
  using IDs = std::vector<ID>;
  using Camera = sensors::Camera;

  using Image = cv_bridge::CvImageConstPtr;
  using Track = std::tuple<Image, variables::Pixel<TScalar>>;
  using Lengths = std::vector<Index>;
  using Positions = std::vector<variables::Position<TScalar>>;

  /// Constructor from time and sensor.
  /// \param time Time.
  /// \param camera Camera.
  VisualTracks(const Time& time, const Camera& camera)
      : VisualTracks{Base::Type::VISUAL_TRACKS, time, camera} {}

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] auto sensor() const -> const Camera& final { return *camera_; }

  /// Sets the associated sensor.
  /// \param camera Sensor to set.
  auto setSensor(const Camera& camera) -> void { camera_ = &camera; }

  /// Adds a new track.
  /// \param camera Associated camera.
  auto addTrack(const Camera& camera) -> Track& {
    const auto [itr, inserted] = tracks_.try_emplace(&camera);
    DCHECK(inserted) << "Track already exists.";
    return itr->second;
  }

  /// Track accessor.
  /// \param camera Associated camera.
  /// \return Track containing the associated image and tracked points.
  [[nodiscard]] auto track(const Camera& camera) const -> const Track& {
    const auto itr = tracks_.find(&camera);
    DCHECK(itr != tracks_.cend()) << "Track does not exist.";
    return itr->second;
  }

  /// Track modifier.
  /// \param camera Associated camera.
  /// \return Track containing the associated image and tracked points.
  auto track(const Camera& camera) -> Track& {
    return const_cast<Track&>(std::as_const(*this).getTrack(camera));
  }

  IDs ids;              ///< Track IDs.
  Lengths lengths;      ///< Track lengths.
  Positions positions;  ///< Track positions (optional).

 protected:
  /// Constructor from message type, time and sensor.
  /// \param type Message type.
  /// \param time Time.
  /// \param camera Camera.
  VisualTracks(const Type& type, const Time& time, const Camera& camera)
      : Base{type, time},
        ids{},
        lengths{},
        positions{},
        camera_{&camera},
        tracks_{} {
    addTrack(camera);
  }

 private:
  const Camera* camera_;  ///< Camera.

  std::map<const Camera*, Track> tracks_;  ///< Tracks.
};

template <typename TScalar>
class StereoVisualTracks : public VisualTracks<TScalar> {
  // Definitions.
  using Base = VisualTracks<TScalar>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;
  using Camera = typename Base::Camera;

  /// Constructor from time and sensor.
  /// \param time Time.
  /// \param camera Camera.
  /// \param other_camera Other camera.
  StereoVisualTracks(const Time& time, const Camera& camera,
                     const Camera& other_camera)
      : Base{Type::STEREO_VISUAL_TRACKS, time, camera},
        other_camera_{&other_camera} {
    this->addTrack(other_camera);
  }

  /// Other sensor accessor.
  /// \return Other sensor.
  [[nodiscard]] auto otherSensor() const -> const Camera& {
    return *other_camera_;
  }

  /// Sets the associated other sensor.
  /// \param other_camera Other sensor to set.
  auto setOtherSensor(const Camera& other_camera) -> void {
    other_camera_ = &other_camera;
  }

 private:
  const Camera* other_camera_;  ///< Other camera.
};

}  // namespace hyper::messages
