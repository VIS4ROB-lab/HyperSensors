/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/measurements/forward.hpp"

#include "hyper/messages/message.hpp"
#include "hyper/variables/variable.hpp"

namespace hyper::messages {

class Measurement : public Message {
 public:
  /// Value accessor.
  /// \return Value.
  [[nodiscard]] virtual auto value() const -> const variables::Variable& = 0;

  /// Value modifier.
  /// \return Value.
  [[nodiscard]] virtual auto value() -> variables::Variable& = 0;

 protected:
  /// Constructor from type and time.
  /// \param type Message type.
  /// \param time Message time.
  explicit Measurement(const Type& type, const Time& time) : Message{type, time} {}
};

template <typename TValue>
class MeasurementBase : public Measurement {
 public:
  // Definitions.
  using Value = TValue;

  /// Value accessor.
  /// \return Value.
  [[nodiscard]] inline auto value() const -> const TValue& final { return value_; }

  /// Value modifier.
  /// \return Value.
  inline auto value() -> TValue& final { return const_cast<TValue&>(std::as_const(*this).value()); }

 protected:
  /// Constructor from type, time and value.
  /// \param type Message type.
  /// \param time Message time.
  /// \param value Value.
  MeasurementBase(const Type& type, const Time& time, const TValue& value) : Measurement{type, time}, value_{value} {}

 private:
  TValue value_;  ///< Value.
};

}  // namespace hyper::messages
