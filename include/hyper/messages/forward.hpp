/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

namespace hyper::messages {

template <typename TScalar>
class Message;

template <typename TScalar>
class VisualTracks;

template <typename TScalar>
using MonocularVisualTracks = VisualTracks<TScalar>;

template <typename TScalar>
class StereoVisualTracks;

}  // namespace hyper::messages
