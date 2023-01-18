/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <glog/logging.h>

#include "hyper/messages/visual.hpp"

namespace hyper::messages {

VisualTracks::VisualTracks(const Stamp& stamp, const Camera& camera)
    : AbstractMessage{stamp, camera},
      tracks{},
      identifiers{},
      positions{},
      lengths{} {
  // Add track.
  addTrack(camera);
}

auto VisualTracks::sensor() const -> const Camera& {
  return static_cast<const Camera&>(*sensor_); // NOLINT
}

auto VisualTracks::setSensor(const Camera& camera) -> void {
  sensor_ = &camera;
}

auto VisualTracks::addTrack(const Camera& camera) -> Entry& {
  const auto [itr, inserted] = tracks.try_emplace(&camera);
  DCHECK(inserted) << "Track already exists.";
  return itr->second;
}

auto VisualTracks::getTrack(const Camera& camera) const -> const Entry& {
  const auto itr = tracks.find(&camera);
  DCHECK(itr != tracks.cend()) << "Track does not exist.";
  return itr->second;
}

auto VisualTracks::getTrack(const Camera& camera) -> Entry& {
  return const_cast<Entry&>(std::as_const(*this).getTrack(camera));
}

StereoVisualTracks::StereoVisualTracks(const Stamp& stamp, const Camera& camera, const Camera& other_camera)
    : VisualTracks{stamp, camera},
      other_sensor_{&other_camera} {
  // Add track.
  addTrack(other_camera);
}

auto StereoVisualTracks::otherSensor() const -> const Camera& {
  return static_cast<const Camera&>(*other_sensor_); // NOLINT
}

auto StereoVisualTracks::setOtherSensor(const Camera& other_camera) -> void {
  other_sensor_ = &other_camera;
}

} // namespace hyper::messages
