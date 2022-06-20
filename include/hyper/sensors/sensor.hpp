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
  using Parameters = CompositeVariable<Scalar>;

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

  /// Parameters accessor.
  /// \return Parameters.
  [[nodiscard]] auto parameters() const -> const Parameters&;

  /// Collects the memory blocks.
  /// \return Memory blocks.
  [[nodiscard]] virtual auto memoryBlocks() const -> MemoryBlocks<Scalar>;

  /// Collects the memory blocks (stamp-based).
  /// \param stamp Query stamp.
  /// \return Memory blocks.
  [[nodiscard]] virtual auto memoryBlocks(const Stamp& stamp) const -> MemoryBlocks<Scalar>;

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
  /// Constructor from number of parameters and YAML file.
  /// \param num_parameters Number of parameters.
  /// \param node Input YAML node.
  Sensor(const Size& num_parameters, const Node& node);

  /// Retrieves the address of a variable.
  /// \param index Index of variable.
  /// \return Address.
  [[nodiscard]] auto address(const Size& index) const -> const Scalar*;

  /// Retrieves the address of a variable.
  /// \param index Index of variable.
  /// \return Address.
  auto address(const Size& index) -> Scalar*;

  /// Outputs all sensor parameters to a YAML emitter.
  /// \param emitter Output YAML emitter.
  virtual auto writeParameters(Emitter& emitter) const -> void;

  Rate rate_;             ///< Rate.
  Parameters parameters_; ///< Parameters.

 private:
  /// Initializes the parameters.
  auto initializeParameters() -> void;

  /// Reads all sensor parameters from a YAML node.
  /// \param node Input YAML node.
  auto readParameters(const Node& node) -> void;
};

} // namespace hyper
