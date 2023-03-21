/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#pragma once

#include "hyper/sensors/forward.hpp"

#include "hyper/variables/groups/se3.hpp"
#include "hyper/yaml/yaml.hpp"

namespace hyper::sensors {

class Sensor {
 public:
  // Parameter block partition.
  template <typename TSize_, typename TPointer_>
  struct Partition {
    TSize_ offset;
    std::vector<TPointer_> parameter_blocks;
    std::vector<TSize_> parameter_block_sizes;
  };

  // Parameter block partitions.
  template <typename TPointer_>
  using Partitions = std::vector<Partition<int, TPointer_>>;

  // Constants.
  static constexpr auto kVariablesOffset = 0;
  static constexpr auto kVariablesPartitionIndex = 0;
  static constexpr auto kNumPartitions = kVariablesPartitionIndex + 1;

  static constexpr auto kOffsetIndex = 0;
  static constexpr auto kTransformationIndex = kOffsetIndex + 1;
  static constexpr auto kNumVariables = kTransformationIndex + 1;

  static constexpr auto kDefaultJacobianType = JacobianType::TANGENT_TO_MANIFOLD;

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

  /// Constructor from Jacobian type.
  /// \param jacobian_type Jacobian type.
  explicit Sensor(JacobianType jacobian_type = kDefaultJacobianType);

  /// Constructor from YAML node.
  /// \param node YAML node.
  explicit Sensor(const Node& node);

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

  /// Type accessor.
  /// \return Sensor type.
  [[nodiscard]] auto type() const -> Type;

  /// Jacobian type accessor.
  /// \return Jacobian type.
  [[nodiscard]] auto jacobianType() const -> JacobianType;

  /// Jacobian type setter.
  /// \param jacobian_type Jacobian type.
  auto setJacobianType(JacobianType jacobian_type) -> void;

  /// Rate accessor.
  /// \return Acquisition rate.
  [[nodiscard]] auto rate() const -> const Rate&;

  /// Checks whether sensor has variable rate.
  /// \return True if rate is variable.
  [[nodiscard]] auto hasVariableRate() const -> bool;

  /// Time-based parameter blocks accessor.
  /// \return Time-based pointers to parameter blocks.
  [[nodiscard]] virtual auto partitions(const Time& time) const -> Partitions<Scalar*>;

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
  /// Definitions.
  using Size = std::size_t;

  /// Constructor from sensor type and number of variables.
  /// \param type Sensor type.
  /// \param jacobian_type Jacobian type.
  /// \param num_variables Number of variables.
  explicit Sensor(Type type, JacobianType jacobian_type, Size num_variables);

  /// Updates the sensor parameter block sizes.
  auto updateSensorParameterBlockSizes() -> void;

  /// Updates the parameter block sizes.
  virtual auto updateParameterBlockSizes() -> void;

  /// Reads a sensor from a YAML node.
  /// \param node YAML node.
  virtual auto read(const Node& node) -> void;

  /// Writes a sensor to a YAML emitter.
  /// \param emitter YAML emitter.
  virtual auto write(Emitter& emitter) const -> void;

  /// Assembles the variables partition.
  /// \return Variables partition
  auto assembleVariablesPartition() const -> Partition<int, Scalar*>;

  Type type_;                   ///< Type.
  JacobianType jacobian_type_;  ///< Jacobian type.

  Rate rate_;                               ///< Rate.
  Variables variables_;                     ///< Variables.
  std::vector<Scalar*> parameter_blocks_;   ///< Parameter blocks.
  std::vector<int> parameter_block_sizes_;  ///< Parameter block sizes.
};

}  // namespace hyper::sensors
