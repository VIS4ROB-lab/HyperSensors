/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/forward.hpp"

#include "hyper/variables/composite.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper {

class Sensor {
 public:
  // Definitions.
  using Size = std::size_t;
  using Node = YAML::Node;
  using Emitter = YAML::Emitter;
  using Parameter = AbstractVariable<Scalar>;
  using Variables = std::vector<std::unique_ptr<Parameter>>;

  using Rate = Traits<Sensor>::Rate;
  using Transformation = Traits<Sensor>::Transformation;

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

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters() const -> Pointers<Parameter>;

  /// Parameters accessor (stamp-based).
  /// \param stamp Query stamp.
  /// \return Parameters.
  [[nodiscard]] virtual auto parameters(const Stamp& stamp) const -> Pointers<Parameter>;

  /// Accesses the transformation.
  /// \return Transformation.
  [[nodiscard]] auto transformation() const -> Eigen::Map<const Transformation>;

  /// Accesses the transformation.
  /// \return Transformation.
  [[nodiscard]] auto transformation() -> Eigen::Map<Transformation>;

  /// Emits a sensor to YAML.
  /// \param emitter Modifiable YAML emitter.
  /// \param sensor Sensor to output.
  /// \return Modified YAML emitter.
  friend auto operator<<(Emitter& emitter, const Sensor& sensor) -> Emitter&;

 protected:
  /// Constructor from number of variables and YAML file.
  /// \param num_variables Number of variables.
  /// \param node Input YAML node.
  Sensor(const Size& num_variables, const Node& node);

  /// Maps a variable in vector form.
  /// \param index Index of variable.
  /// \return Variable as vector.
  [[nodiscard]] auto variableAsVector(const Size& index) const -> Eigen::Map<DynamicVector<Scalar>>;

  /// Outputs all sensor variables to a YAML emitter.
  /// \param emitter Output YAML emitter.
  virtual auto writeVariables(Emitter& emitter) const -> void;

  Rate rate_;           ///< Rate.
  Variables variables_; ///< Variables.

 private:
  /// Initializes the variables.
  auto initializeVariables() -> void;

  /// Reads all sensor variables from a YAML node.
  /// \param node Input YAML node.
  auto readVariables(const Node& node) -> void;
};

} // namespace hyper
