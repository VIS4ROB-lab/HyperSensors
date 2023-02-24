/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <utility>

#include <Eigen/Core>

#include "hyper/messages/forward.hpp"
#include "hyper/sensors/forward.hpp"

namespace hyper::messages {

template <typename TScalar>
class Message {
 public:
  // Enum.
  enum class Type {
    VISUAL_TRACKS,
    STEREO_VISUAL_TRACKS,
    ABSOLUTE_MEASUREMENT,
    VISUAL_MEASUREMENT,
    INERTIAL_MEASUREMENT,
    RELATIVE_MEASUREMENT,
  };

  // Definitions.
  using ID = Eigen::Index;
  using Size = Eigen::Index;

  using Time = TScalar;
  using Scalar = TScalar;
  using Sensor = sensors::Sensor;

  /// Default destructor.
  virtual ~Message() = default;

  /// Downcasts this instance.
  /// \tparam TDerived_ Target type.
  /// \return Cast instance.
  template <typename TDerived_>
  inline auto as() const -> const TDerived_& {
    return static_cast<const TDerived_&>(*this);
  }

  /// Downcasts this instance.
  /// \tparam TDerived_ Target type.
  /// \return Cast instance.
  template <typename TDerived_>
  inline auto as() -> TDerived_& {
    return const_cast<TDerived_&>(
        std::as_const(*this).template as<TDerived_>());
  }

  /// Type accessor.
  /// \return Message type.
  [[nodiscard]] inline auto type() const -> const Type& { return type_; }

  /// Time accessor.
  /// \return Time.
  [[nodiscard]] inline auto time() const -> const Time& { return time_; }

  /// Time modifier.
  /// \return Time.
  inline auto time() -> Time& {
    return const_cast<Time&>(std::as_const(*this).time());
  }

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] virtual auto sensor() const -> const Sensor* = 0;

 protected:
  /// Constructor from type and time.
  /// \param type Message type.
  /// \param time Message time.
  explicit Message(const Type& type, const Time& time)
      : type_{type}, time_{time} {}

 private:
  Type type_;  ///< Type.
  Time time_;  ///< Time.
};

}  // namespace hyper::messages
