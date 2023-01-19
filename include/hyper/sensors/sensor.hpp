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

  /// Constructor from YAML file.
  /// \param node Input YAML node.
  explicit Sensor(const Node& node = {});

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
  [[nodiscard]] auto offset() const -> Eigen::Map<const Offset>;

  /// Offset modifier.
  /// \return Offset.
  [[nodiscard]] auto offset() -> Eigen::Map<Offset>;

  /// Transformation accessor.
  /// \return Transformation.
  [[nodiscard]] auto transformation() const -> Eigen::Map<const Transformation>;

  /// Transformation modifier.
  /// \return Transformation.
  [[nodiscard]] auto transformation() -> Eigen::Map<Transformation>;

  /// Emits a sensor to YAML.
  /// \param emitter Modifiable YAML emitter.
  /// \param sensor Sensor to output.
  /// \return Modified YAML emitter.
  friend auto operator<<(Emitter& emitter, const Sensor& sensor) -> Emitter&;

 protected:
  // Definitions.
  using Index = std::size_t;

  /// Constructor from number of variables and YAML file.
  /// \param num_variables Number of variables.
  /// \param node Input YAML node.
  Sensor(const Index& num_variables, const Node& node);

  /// Writes the sensor information to a YAML emitter.
  /// \param emitter Output YAML emitter.
  virtual auto write(Emitter& emitter) const -> void;

  Rate rate_;                        ///< Rate.
  Variables variables_;              ///< Variables.
  std::vector<Scalar*> parameters_;  ///< Parameters (i.e. pointer to variable memory).

 private:
  /// Reads the sensor information from a YAML node.
  /// \param node Input YAML node.
  auto read(const Node& node) -> void;
};

}  // namespace hyper::sensors
