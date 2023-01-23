/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/forward.hpp"

#include "hyper/variables/groups/se3.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper::sensors {

class Sensor {
 public:
  // Constants.
  static constexpr auto kOffsetOffset = 0;
  static constexpr auto kTransformationOffset = kOffsetOffset + 1;
  static constexpr auto kNumVariables = kTransformationOffset + 1;

  // Definitions.
  using Node = YAML::Node;
  using Emitter = YAML::Emitter;

  using Time = double;
  using Scalar = double;

  using Variable = variables::Variable<Scalar>;
  using Variables = std::vector<std::unique_ptr<Variable>>;

  using Rate = Scalar;
  using Offset = variables::Cartesian<Scalar, 1>;
  using Transformation = variables::SE3<Scalar>;

  /// Default constructor.
  Sensor();

  /// Default destructor.
  virtual ~Sensor() = default;

  /// Downcast this instance.
  /// \tparam TDerived_ Target type.
  /// \return Reference to cast instance.
  template <typename TDerived_>
  inline auto as() const -> const TDerived_& {
    return static_cast<const TDerived_&>(*this);
  }

  /// Downcast this instance.
  /// \tparam TDerived_ Target type.
  /// \return Reference to cast instance.
  template <typename TDerived_>
  inline auto as() -> TDerived_& {
    return const_cast<TDerived_&>(std::as_const(*this).template as<TDerived_>());
  }

  /// \brief Rate accessor.
  /// \return Acquisition rate.
  [[nodiscard]] auto rate() const -> const Rate&;

  /// Checks whether sensor has variable rate.
  /// \return True if rate is variable.
  [[nodiscard]] auto hasVariableRate() const -> bool;

  /// Variables accessor.
  /// \return Variables.
  [[nodiscard]] auto variables() const -> const Variables&;

  /// Pointers accessor.
  /// \return Pointers.
  [[nodiscard]] virtual auto pointers() const -> std::vector<Variable*>;

  /// Pointers accessor (time-based).
  /// \param time Query time.
  /// \return Pointers.
  [[nodiscard]] virtual auto pointers(const Time& time) const -> std::vector<Variable*>;

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters() const -> std::vector<Scalar*>;

  /// Parameters accessor (time-based).
  /// \param time Query time.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters(const Time& time) const -> std::vector<Scalar*>;

  /// Offset accessor.
  /// \return Offset.
  [[nodiscard]] auto offset() const -> const Offset&;

  /// Offset modifier.
  /// \return Offset.
  [[nodiscard]] auto offset() -> Offset&;

  /// Transformation accessor.
  /// \return Transformation.
  [[nodiscard]] auto transformation() const -> const Transformation&;

  /// Transformation modifier.
  /// \return Transformation.
  [[nodiscard]] auto transformation() -> Transformation&;

  /// Reads a sensor from a YAML file.
  /// \param node YAML node.
  /// \param sensor Sensor to read.
  /// \return YAML node.
  friend auto operator>>(const Node& node, Sensor& sensor) -> const Node&;

  /// Emits a sensor to a YAML file.
  /// \param emitter YAML emitter.
  /// \param sensor Sensor to emit.
  /// \return Modified YAML emitter.
  friend auto operator<<(Emitter& emitter, const Sensor& sensor) -> Emitter&;

 protected:
  // Definitions.
  using Size = std::size_t;

  /// Constructor from number of variables.
  /// \param num_variables Number of variables.
  explicit Sensor(const Size& num_variables);

  /// Reads a sensor from a YAML node.
  /// \param node YAML node.
  virtual auto read(const Node& node) -> void;

  /// Writes a sensor to a YAML emitter.
  /// \param emitter YAML emitter.
  virtual auto write(Emitter& emitter) const -> void;

  Rate rate_;                        ///< Rate.
  Variables variables_;              ///< Variables.
  std::vector<Scalar*> parameters_;  ///< Parameters (i.e. pointer to variable memory).
};

}  // namespace hyper::sensors
