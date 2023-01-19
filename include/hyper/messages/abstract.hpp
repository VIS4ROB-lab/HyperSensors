/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include <utility>

#include "hyper/messages/forward.hpp"
#include "hyper/sensors/forward.hpp"

namespace hyper::messages {

class AbstractMessage {
 public:
  // Definitions.
  using Time = double;
  using Scalar = double;
  using Sensor = sensors::Sensor;

  /// Default destructor.
  virtual ~AbstractMessage() = default;

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
    return const_cast<TDerived_&>(std::as_const(*this).template as<TDerived_>());
  }

  /// Time accessor.
  /// \return Time.
  [[nodiscard]] auto time() const -> const Time&;

  /// Time modifier.
  /// \return Time.
  auto time() -> Time&;

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] virtual auto sensor() const -> const Sensor&;

 protected:
  /// Constructor from time and sensor.
  /// \param time Time.
  /// \param sensor Sensor.
  explicit AbstractMessage(const Time& time, const Sensor& sensor);

  Time time_;             ///< Time.
  const Sensor* sensor_;  ///< Sensor.
};

}  // namespace hyper::messages
