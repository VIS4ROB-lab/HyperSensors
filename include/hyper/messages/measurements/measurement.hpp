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
  explicit Measurement(const Type& type, const Time& time) : Base{type, time} {}
};

template <typename TVariable>
class MeasurementBase : public Measurement<typename TVariable::Scalar> {
 public:
  // Definitions.
  using Base = Measurement<typename TVariable::Scalar>;
  using Type = typename Base::Type;
  using Time = typename Base::Time;

  /// Variable accessor.
  /// \return Variable.
  [[nodiscard]] inline auto variable() const -> const TVariable& final {
    return variable_;
  }

  /// Variable modifier.
  /// \return Variable.
  inline auto variable() -> TVariable& final {
    return const_cast<TVariable&>(std::as_const(*this).variable());
  }

 protected:
  /// Constructor from type, time and variable.
  /// \param type Message type.
  /// \param time Message time.
  /// \param variable Variable.
  MeasurementBase(const Type& type, const Time& time, const TVariable& variable)
      : Base{type, time}, variable_{variable} {}

 private:
  TVariable variable_;  ///< Variable.
};

}  // namespace hyper::messages
