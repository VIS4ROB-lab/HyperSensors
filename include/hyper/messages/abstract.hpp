/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/definitions.hpp"
#include "hyper/messages/forward.hpp"
#include "hyper/sensors/forward.hpp"

namespace hyper::messages {

class AbstractMessage {
 public:
  // Definitions.
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

  /// Stamp accessor.
  /// \return Stamp.
  [[nodiscard]] auto stamp() const -> const Stamp&;

  /// Stamp modifier.
  /// \return Stamp.
  auto stamp() -> Stamp&;

  /// Sensor accessor.
  /// \return Sensor.
  [[nodiscard]] virtual auto sensor() const -> const Sensor&;

 protected:
  /// Constructor from stamp and sensor.
  /// \param stamp Stamp.
  /// \param sensor Sensor.
  explicit AbstractMessage(const Stamp& stamp, const Sensor& sensor);

  Stamp stamp_;          ///< Stamp.
  const Sensor* sensor_; ///< Sensor.
};

} // namespace hyper::messages
