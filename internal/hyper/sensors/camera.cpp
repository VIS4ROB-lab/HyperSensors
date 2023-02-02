/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include "hyper/sensors/camera.hpp"
#include "hyper/variables/jacobian.hpp"

namespace hyper::sensors {

namespace {

// Variable names.
constexpr auto kSensorSizeName = "sensor_size";
constexpr auto kShutterTypeName = "shutter_type";
constexpr auto kShutterDeltaName = "shutter_delta";
constexpr auto kIntrinsicsName = "intrinsics";
constexpr auto kDistortionName = "distortion";

// Default parameters.
constexpr auto kDefaultShutterDelta = 0;

}  // namespace

auto Camera::LandmarkToPixel(const Eigen::Ref<const Landmark>& landmark, Scalar* J_l) -> Pixel {
  const auto inverse_z = Scalar{1} / landmark.z();

  if (J_l) {
    using Jacobian = variables::JacobianNM<Pixel, Landmark>;
    auto J = Eigen::Map<Jacobian>{J_l};
    const auto inverse_z2 = inverse_z * inverse_z;
    J(0, 0) = inverse_z;
    J(1, 0) = Scalar{0};
    J(0, 1) = Scalar{0};
    J(1, 1) = inverse_z;
    J(0, 2) = -landmark.x() * inverse_z2;
    J(1, 2) = -landmark.y() * inverse_z2;
  }

  return {landmark.x() * inverse_z, landmark.y() * inverse_z};
}

auto Camera::LandmarkToBearing(const Eigen::Ref<const Landmark>& landmark, Scalar* J_l) -> Bearing {
  const auto i_n2 = Scalar{1} / landmark.squaredNorm();
  const auto i_n = std::sqrt(i_n2);

  if (J_l) {
    using Jacobian = variables::JacobianNM<Bearing, Landmark>;
    Eigen::Map<Jacobian>{J_l} = (Jacobian::Identity() - landmark * landmark.transpose() * i_n2) * i_n;
  }

  return landmark * i_n;
}

auto Camera::PixelToBearing(const Eigen::Ref<const Pixel>& pixel, Scalar* J_p) -> Bearing {
  const auto x = pixel.x();
  const auto y = pixel.y();
  const auto x2 = x * x;
  const auto y2 = y * y;
  const auto i_n2 = Scalar{1} / (Scalar{1} + x2 + y2);
  const auto i_n = std::sqrt(i_n2);

  if (J_p) {
    using Jacobian = variables::JacobianNM<Bearing, Pixel>;
    auto J = Eigen::Map<Jacobian>{J_p};
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

auto Camera::Triangulate(const Eigen::Ref<const Transformation>& T_ab, const Eigen::Ref<const Bearing>& b_a, const Eigen::Ref<const Bearing>& b_b) -> Landmark {
  // Triangulate landmark.
  const auto c0 = (T_ab.rotation() * b_b).eval();
  const auto c1 = c0.cross(b_a).norm();
  const auto c2 = c0.cross(T_ab.translation()).norm();
  const auto c3 = b_a.cross(T_ab.translation()).norm();
  return c2 / (c2 + c3) * (T_ab.translation() + (c3 / c1) * (b_a + c0));
}

Camera::Camera() : Sensor{Type::CAMERA, kNumVariables}, sensor_size_{}, shutter_type_{ShutterType::DEFAULT}, shutter_delta_{kDefaultShutterDelta} {
  // Initialize variables.
  DCHECK_LE(kNumVariables, variables_.size());
  variables_[kIntrinsicsIndex] = std::make_unique<Intrinsics>();
  parameter_blocks_[kIntrinsicsIndex] = variables_[kIntrinsicsIndex]->asVector().data();
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

auto Camera::intrinsics() const -> const Intrinsics& {
  return static_cast<const Intrinsics&>(*variables_[kIntrinsicsIndex]);  // NOLINT
}

auto Camera::intrinsics() -> Intrinsics& {
  return const_cast<Intrinsics&>(std::as_const(*this).intrinsics());
}

auto Camera::distortion() const -> const Distortion& {
  return static_cast<const Distortion&>(*variables_[kDistortionIndex]);  // NOLINT
}

auto Camera::distortion() -> Distortion& {
  return const_cast<Distortion&>(std::as_const(*this).distortion());
}

auto Camera::setDistortion(std::unique_ptr<Distortion>&& distortion) -> void {
  CHECK(distortion != nullptr);
  variables_[kDistortionIndex] = std::move(distortion);
  parameter_blocks_[kDistortionIndex] = variables_[kDistortionIndex]->asVector().data();
}

auto Camera::randomPixel() const -> Pixel {
  Pixel pixel;
  pixel.x() = static_cast<Scalar>(sensor_size_.width - 1) * Eigen::internal::random<Scalar>(0, 1);
  pixel.y() = static_cast<Scalar>(sensor_size_.height - 1) * Eigen::internal::random<Scalar>(0, 1);
  return pixel;
}

auto Camera::randomNormalizedPixel(bool distort) const -> Pixel {
  if (distort) {
    const auto normalized_pixel = intrinsics().normalize(randomPixel());
    return distortion().distort(normalized_pixel, nullptr, nullptr, nullptr);
  } else {
    return intrinsics().normalize(randomPixel());
  }
}

auto Camera::randomBearing(bool distorted) -> Bearing {
  return PixelToBearing(randomNormalizedPixel(distorted));
}

auto Camera::correctShutterTimes(const Time& time, const std::vector<Pixel>& pixels) const -> std::vector<Time> {
  std::vector<Time> times;
  times.reserve(pixels.size());
  if (shutterType() == ShutterType::HORIZONTAL) {
    const auto h_2 = Scalar{0.5} * static_cast<Scalar>(sensorSize().height - 1);
    for (const auto& pixel : pixels) {
      times.emplace_back(time + shutter_delta_ * (pixel.y() - h_2));
    }
  } else {  // Vertical rolling shutter.
    const auto w_2 = Scalar{0.5} * static_cast<Scalar>(sensorSize().width - 1);
    for (const auto& pixel : pixels) {
      times.emplace_back(time + shutter_delta_ * (pixel.x() - w_2));
    }
  }
  return times;
}

auto Camera::pixelsToBearings(const std::vector<Pixel>& pixels, const Scalar* parameters) const -> std::vector<Bearing> {
  // Allocate memory.
  std::vector<Bearing> bearings;
  bearings.reserve(pixels.size());

  // Undistort and convert.
  for (const auto& pixel : pixels) {
    const auto normalized_pixel = intrinsics().normalize(pixel);
    const auto undistorted_pixel = distortion().undistort(normalized_pixel, nullptr, nullptr, parameters);
    bearings.emplace_back(PixelToBearing(undistorted_pixel));
  }

  return bearings;
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

auto Camera::ReadDistortion(const Node& node) -> std::unique_ptr<Distortion> {
  const auto distortion_node = yaml::Read(node, kDistortionName);
  const auto distortion_type_string = yaml::ReadAs<std::string>(distortion_node, "type");
  if (distortion_type_string == "RADIAL_TANGENTIAL_2") {
    using RadialTangentialDistortion = variables::RadialTangentialDistortion<Scalar, 2>;
    const auto distortion = yaml::ReadVariable<RadialTangentialDistortion>(distortion_node, "parameters");
    return std::make_unique<RadialTangentialDistortion>(distortion);
  } else {
    LOG(FATAL) << "Unknown distortion type.";
    return nullptr;
  }
}

auto Camera::read(const Node& node) -> void {
  Sensor::read(node);
  sensor_size_ = ReadSensorSize(node);
  shutter_type_ = ReadShutterType(node);
  shutter_delta_ = (shutterType() != ShutterType::GLOBAL) ? ReadShutterDelta(node) : kDefaultShutterDelta;
  intrinsics() = yaml::ReadVariable<Intrinsics>(node, kIntrinsicsName);
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
  emitter << YAML::Key << kDistortionName;
  emitter << YAML::Value << YAML::BeginMap;
  yaml::Write(emitter, "type", "RADIAL_TANGENTIAL_2");
  yaml::WriteVariable(emitter, "parameters", distortion());
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;
}

auto Camera::write(Emitter& emitter) const -> void {
  Sensor::write(emitter);
  writeSensorSize(emitter);
  writeShutterType(emitter);
  writeShutterDelta(emitter);
  yaml::WriteVariable(emitter, kIntrinsicsName, intrinsics());
  writeDistortion(emitter);
}

}  // namespace hyper::sensors
