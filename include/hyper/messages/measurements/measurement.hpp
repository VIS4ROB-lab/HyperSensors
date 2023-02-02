/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/messages/message.hpp"
#include "hyper/variables/variable.hpp"

namespace hyper::messages {

template <typename TScalar>
class Measurement : public Message<TScalar> {
 public:
  // Definitions.
  using Base = Message<TScalar>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  using Variable = variables::Variable<TScalar>;

  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] virtual auto variable() const -> const Variable& = 0;

  /// Variable modifier.
  /// \return Variable.
  [[nodiscard]] virtual auto variable() -> Variable& = 0;

 protected:
  /// Constructor from type and time.
  /// \param type Message type.
  /// \param time Message time.
  explicit Measurement(const Type& type, const Time& time)
      : Base{type, time} {}
};

}  // namespace hyper::messages
