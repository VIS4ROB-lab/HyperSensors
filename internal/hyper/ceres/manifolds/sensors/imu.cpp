/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#ifdef HYPER_COMPILE_WITH_CERES

#include "hyper/ceres/manifolds/sensors/imu.hpp"
#include "hyper/ceres/manifolds/variables/euclidean.hpp"
#include "hyper/ceres/manifolds/variables/stamped.hpp"
#include "hyper/state/continuous.hpp"

namespace hyper::ceres::manifolds {

using namespace sensors;

Manifold<IMU>::Manifold(const IMU* imu, const bool constant) : Manifold<Sensor>{imu, kNumSubmanifolds, constant} {
  setGyroscopeIntrinsicsConstant(constant);
  setAccelerometerIntrinsicsConstant(constant);
  setGyroscopeBiasConstant(constant);
  setAccelerometerBiasConstant(constant);
}

auto Manifold<IMU>::sensor() const -> const IMU* {
  return static_cast<const IMU*>(sensor_);  // NOLINT
}

auto Manifold<IMU>::gyroscopeIntrinsicsSubmanifold() const -> Submanifold* {
  return submanifolds_[IMU::kGyroscopeIntrinsicsIndex].get();
}

auto Manifold<IMU>::setGyroscopeIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::GyroscopeIntrinsics::kNumParameters);
  submanifolds_[IMU::kGyroscopeIntrinsicsIndex] = std::move(submanifold);
}

auto Manifold<IMU>::setGyroscopeIntrinsicsConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::GyroscopeIntrinsics>>(constant);
  setGyroscopeIntrinsicsSubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::gyroscopeSensitivitySubmanifold() const -> Submanifold* {
  return submanifolds_[IMU::kGyroscopeSensitivityIndex].get();
}

auto Manifold<IMU>::setGyroscopeSensitivitySubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::GyroscopeSensitivity::kNumParameters);
  submanifolds_[IMU::kGyroscopeSensitivityIndex] = std::move(submanifold);
}

auto Manifold<IMU>::setGyroscopeSensitivityConstant(bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::GyroscopeSensitivity>>(constant);
  setGyroscopeSensitivitySubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::gyroscopeBiasSubmanifold() const -> Submanifold* {
  return submanifolds_[kGyroscopeBiasSubmanifoldIndex].get();
}

auto Manifold<IMU>::setGyroscopeBiasSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::GyroscopeBias::StampedVariable::kNumParameters);
  submanifolds_[kGyroscopeBiasSubmanifoldIndex] = std::move(submanifold);
}

auto Manifold<IMU>::setGyroscopeBiasConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::GyroscopeBias::StampedVariable>>(true, constant);
  setGyroscopeBiasSubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::accelerometerIntrinsicsSubmanifold() const -> Submanifold* {
  return submanifolds_[IMU::kAccelerometerIntrinsicsIndex].get();
}

auto Manifold<IMU>::setAccelerometerIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::AccelerometerIntrinsics::kNumParameters);
  submanifolds_[IMU::kAccelerometerIntrinsicsIndex] = std::move(submanifold);
}

auto Manifold<IMU>::setAccelerometerIntrinsicsConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::AccelerometerIntrinsics>>(constant);
  setAccelerometerIntrinsicsSubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::accelerometerOffsetSubmanifold() const -> Submanifold* {
  return submanifolds_[IMU::kAccelerometerOffsetIndex].get();
}

auto Manifold<IMU>::setAccelerometerOffsetSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::AccelerometerOffset::kNumParameters);
  submanifolds_[IMU::kAccelerometerOffsetIndex] = std::move(submanifold);
}

auto Manifold<IMU>::setAccelerometerOffsetConstant(bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::AccelerometerOffset>>(constant);
  setAccelerometerOffsetSubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::accelerometerBiasSubmanifold() const -> Submanifold* {
  return submanifolds_[kAccelerometerBiasSubmanifoldIndex].get();
}

auto Manifold<IMU>::setAccelerometerBiasSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::AccelerometerBias::StampedVariable::kNumParameters);
  submanifolds_[kAccelerometerBiasSubmanifoldIndex] = std::move(submanifold);
}

auto Manifold<IMU>::setAccelerometerBiasConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::AccelerometerBias::StampedVariable>>(true, constant);
  setAccelerometerBiasSubmanifold(std::move(submanifold));
}

}  // namespace hyper::ceres::manifolds

#endif
