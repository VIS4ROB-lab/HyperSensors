/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/sensors/camera.hpp"
#include "hyper/variables/distortions/radial_tangential.hpp"
#include "hyper/variables/intrinsics.hpp"
#include "hyper/variables/jacobian.hpp"

namespace hyper::sensors {

namespace {

// Variable names.
constexpr auto kSensorSizeName = "resolution";
constexpr auto kShutterTypeName = "shutter";
constexpr auto kShutterDeltaName = "shutter_delta";
constexpr auto kIntrinsicsName = "intrinsics";
constexpr auto kDistortionName = "distortion";

// Default parameters.
constexpr auto kDefaultShutterDelta = 0;

}  // namespace

auto Camera::ProjectToPlane(const Eigen::Ref<const Position<Scalar>>& position, Scalar* raw_J) -> Pixel<Scalar> {
  const auto inverse_z = Scalar{1} / position.z();

  if (raw_J) {
    using Jacobian = Jacobian<Pixel<Scalar>, Position<Scalar>>;
    auto J = Eigen::Map<Jacobian>{raw_J};
    const auto inverse_z2 = inverse_z * inverse_z;
    J(0, 0) = inverse_z;
    J(1, 0) = Scalar{0};
    J(0, 1) = Scalar{0};
    J(1, 1) = inverse_z;
    J(0, 2) = -position.x() * inverse_z2;
    J(1, 2) = -position.y() * inverse_z2;
  }

  return {position.x() * inverse_z, position.y() * inverse_z};
}

auto Camera::ProjectToSphere(const Eigen::Ref<const Position<Scalar>>& position, Scalar* raw_J) -> Bearing<Scalar> {
  const auto i_n2 = Scalar{1} / position.squaredNorm();
  const auto i_n = std::sqrt(i_n2);

  if (raw_J) {
    using Jacobian = Jacobian<Bearing<Scalar>, Position<Scalar>>;
    Eigen::Map<Jacobian>{raw_J} = (Jacobian::Identity() - position * position.transpose() * i_n2) * i_n;
  }

  return position * i_n;
}

auto Camera::LiftToSphere(const Eigen::Ref<const Pixel<Scalar>>& pixel, Scalar* raw_J) -> Bearing<Scalar> {
  const auto x = pixel.x();
  const auto y = pixel.y();
  const auto x2 = x * x;
  const auto y2 = y * y;
  const auto i_n2 = Scalar{1} / (Scalar{1} + x2 + y2);
  const auto i_n = std::sqrt(i_n2);

  if (raw_J) {
    using Jacobian = Jacobian<Bearing<Scalar>, Pixel<Scalar>>;
    auto J = Eigen::Map<Jacobian>{raw_J};
    const auto xy = pixel.x() * pixel.y();
    const auto i_n3 = i_n * i_n2;
    J(0, 0) = (1 + y2) * i_n3;
    J(1, 0) = -xy * i_n3;
    J(2, 0) = -x * i_n3;
    J(0, 1) = -xy * i_n3;
    J(1, 1) = (1 + x2) * i_n3;
    J(2, 1) = -y * i_n3;
  }

  return {pixel.x() * i_n, pixel.y() * i_n, i_n};
}

auto Camera::Triangulate(const Eigen::Ref<const Transformation>& T_ab, const Eigen::Ref<const Bearing<Scalar>>& b_a, const Eigen::Ref<const Bearing<Scalar>>& b_b)
    -> Position<Scalar> {
  // Triangulate position.
  const auto c0 = (T_ab.rotation() * b_b).eval();
  const auto c1 = c0.cross(b_a).norm();
  const auto c2 = c0.cross(T_ab.translation()).norm();
  const auto c3 = b_a.cross(T_ab.translation()).norm();
  return c2 / (c2 + c3) * (T_ab.translation() + (c3 / c1) * (b_a + c0));
}

Camera::Camera(const Node& node) : Sensor{kNumParameters, node}, sensor_size_{}, shutter_type_{ShutterType::DEFAULT}, shutter_delta_{kDefaultShutterDelta} {
  initializeVariables();
  if (!node.IsNull()) {
    readVariables(node);
  }
}

auto Camera::sensorSize() const -> const SensorSize& {
  return sensor_size_;
}

auto Camera::sensorSize() -> SensorSize& {
  return const_cast<SensorSize&>(std::as_const(*this).sensorSize());
}

auto Camera::shutterType() const -> const ShutterType& {
  return shutter_type_;
}

auto Camera::shutterType() -> ShutterType& {
  return const_cast<ShutterType&>(std::as_const(*this).shutterType());
}

auto Camera::shutterDelta() const -> const ShutterDelta& {
  return shutter_delta_;
}

auto Camera::shutterDelta() -> ShutterDelta& {
  return const_cast<ShutterDelta&>(std::as_const(*this).shutterDelta());
}

auto Camera::intrinsics() const -> Eigen::Map<const Intrinsics<Scalar>> {
  const auto vector = variableAsVector(kIntrinsicsOffset);
  return Eigen::Map<const Intrinsics<Scalar>>{vector.data()};
}

auto Camera::intrinsics() -> Eigen::Map<Intrinsics<Scalar>> {
  auto vector = variableAsVector(kIntrinsicsOffset);
  return Eigen::Map<Intrinsics<Scalar>>{vector.data()};
}

auto Camera::distortion() const -> const AbstractDistortion<Scalar>& {
  const auto p_distortion = variables_[kDistortionOffset].get();
  DCHECK(p_distortion != nullptr);
  return static_cast<const AbstractDistortion<Scalar>&>(*p_distortion);  // NOLINT
}

auto Camera::distortion() -> AbstractDistortion<Scalar>& {
  return const_cast<AbstractDistortion<Scalar>&>(std::as_const(*this).distortion());
}

auto Camera::setDistortion(std::unique_ptr<AbstractDistortion<Scalar>>&& distortion) -> void {
  DCHECK_LE(kNumParameters, variables_.size());
  DCHECK(distortion.get() != nullptr);
  variables_[kDistortionOffset] = std::move(distortion);
}

auto Camera::correctShutterStamps(const Stamp& stamp, const std::vector<Pixel<Scalar>>& pixels) const -> Stamps {
  Stamps stamps;
  stamps.reserve(pixels.size());
  if (shutterType() == ShutterType::HORIZONTAL) {
    const auto h_2 = Scalar{0.5} * sensorSize().height;
    for (const auto& pixel : pixels) {
      stamps.emplace_back(stamp + shutter_delta_ * (pixel.y() - h_2));
    }
  } else {  // Vertical rolling shutter.
    const auto w_2 = Scalar{0.5} * sensorSize().width;
    for (const auto& pixel : pixels) {
      stamps.emplace_back(stamp + shutter_delta_ * (pixel.x() - w_2));
    }
  }
  return stamps;
}

auto Camera::convertPixelsToBearings(const std::vector<Pixel<Scalar>>& pixels) const -> std::vector<Bearing<Scalar>> {
  // Allocate memory.
  std::vector<Bearing<Scalar>> bearings;
  bearings.reserve(pixels.size());

  // Undistort and convert.
  for (const auto& pixel : pixels) {
    const auto normalized_pixel = intrinsics().normalize(pixel);
    const auto undistorted_pixel = distortion().undistort(normalized_pixel, nullptr, nullptr);
    bearings.emplace_back(LiftToSphere(undistorted_pixel));
  }

  return bearings;
}

auto Camera::triangulate(const Camera& other, const Eigen::Ref<const Bearing<Scalar>>& b_this, const Eigen::Ref<const Bearing<Scalar>>& b_other) -> Position<Scalar> {
  const auto T_this_other = transformation().groupInverse().groupPlus(other.transformation());
  return Triangulate(T_this_other, b_this, b_other);
}

auto Camera::initializeVariables() -> void {
  DCHECK_LE(kNumParameters, variables_.size());
  variables_[kIntrinsicsOffset] = std::make_unique<Intrinsics<Scalar>>();
  variables_[kDistortionOffset] = nullptr;
}

auto Camera::ReadSensorSize(const Node& node) -> SensorSize {
  const auto dimensions = yaml::ReadAs<std::vector<Index>>(node, kSensorSizeName);
  CHECK_EQ(dimensions.size(), 2);
  return {dimensions[0], dimensions[1]};
}

auto Camera::ReadShutterType(const Node& node) -> ShutterType {
  const auto shutter_type_string = yaml::ReadAs<std::string>(node, kShutterTypeName);
  if (shutter_type_string == "GLOBAL") {
    return ShutterType::GLOBAL;
  } else if (shutter_type_string == "VERTICAL") {
    return ShutterType::VERTICAL;
  } else if (shutter_type_string == "HORIZONTAL") {
    return ShutterType::HORIZONTAL;
  } else {
    LOG(FATAL) << "Unknown shutter type.";
    return ShutterType::DEFAULT;
  }
}

auto Camera::ReadShutterDelta(const Node& node) -> ShutterDelta {
  return yaml::ReadAs<ShutterDelta>(node, kShutterDeltaName);
}

auto Camera::ReadDistortion(const Node& node) -> std::unique_ptr<AbstractDistortion<Scalar>> {
  const auto distortion_node = yaml::Read(node, kDistortionName);
  const auto distortion_type_string = yaml::ReadAs<std::string>(distortion_node, "type");
  if (distortion_type_string == "radial_tangential") {
    using Distortion = RadialTangentialDistortion<Scalar, 2>;
    const auto distortion = yaml::ReadVariable<Distortion>(distortion_node, "parameters");
    return std::make_unique<Distortion>(distortion);
  } else {
    LOG(FATAL) << "Unknown distortion type.";
    return nullptr;
  }
}

auto Camera::readVariables(const Node& node) -> void {
  sensor_size_ = ReadSensorSize(node);
  shutter_type_ = ReadShutterType(node);
  shutter_delta_ = (shutterType() != ShutterType::GLOBAL) ? ReadShutterDelta(node) : kDefaultShutterDelta;
  intrinsics() = yaml::ReadVariable<Intrinsics<Scalar>>(node, kIntrinsicsName);
  setDistortion(ReadDistortion(node));
}

auto Camera::writeSensorSize(Emitter& emitter) const -> void {
  const auto& sensor_size = sensorSize();
  emitter << YAML::Key << kSensorSizeName << YAML::Value << YAML::Flow << YAML::BeginSeq << sensor_size.width << sensor_size.height << YAML::EndSeq;
}

auto Camera::writeShutterType(Emitter& emitter) const -> void {
  switch (shutterType()) {
    case ShutterType::GLOBAL: {
      yaml::Write(emitter, kShutterTypeName, "GLOBAL");
      return;
    }
    case ShutterType::HORIZONTAL: {
      yaml::Write(emitter, kShutterTypeName, "HORIZONTAL");
      return;
    }
    case ShutterType::VERTICAL: {
      yaml::Write(emitter, kShutterTypeName, "VERTICAL");
      return;
    }
  }
}

auto Camera::writeShutterDelta(Emitter& emitter) const -> void {
  if (shutterType() != ShutterType::GLOBAL) {
    yaml::Write(emitter, kShutterDeltaName, shutterDelta());
  }
}

auto Camera::writeDistortion(Emitter& emitter) const -> void {
  emitter << YAML::BeginMap << YAML::Key << kDistortionName;
  emitter << YAML::Value << YAML::BeginMap;
  yaml::Write(emitter, "type", "radial_tangential");
  yaml::WriteVariable(emitter, "parameters", distortion());
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;
}

auto Camera::writeVariables(Emitter& emitter) const -> void {
  Sensor::writeVariables(emitter);
  writeSensorSize(emitter);
  writeShutterType(emitter);
  writeShutterDelta(emitter);
  yaml::WriteVariable(emitter, kIntrinsicsName, intrinsics());
  writeDistortion(emitter);
}

}  // namespace hyper::sensors
