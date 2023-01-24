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
  return submanifolds_[IMU::kGyroscopeIntrinsicsOffset].get();
}

auto Manifold<IMU>::setGyroscopeIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::GyroscopeIntrinsics::kNumParameters);
  submanifolds_[IMU::kGyroscopeIntrinsicsOffset] = std::move(submanifold);
}

auto Manifold<IMU>::setGyroscopeIntrinsicsConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::GyroscopeIntrinsics>>(constant);
  setGyroscopeIntrinsicsSubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::accelerometerIntrinsicsSubmanifold() const -> Submanifold* {
  return submanifolds_[IMU::kAccelerometerIntrinsicsOffset].get();
}

auto Manifold<IMU>::setAccelerometerIntrinsicsSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::AccelerometerIntrinsics::kNumParameters);
  submanifolds_[IMU::kAccelerometerIntrinsicsOffset] = std::move(submanifold);
}

auto Manifold<IMU>::setAccelerometerIntrinsicsConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::AccelerometerIntrinsics>>(constant);
  setAccelerometerIntrinsicsSubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::gyroscopeBiasSubmanifold() const -> Submanifold* {
  return submanifolds_[kGyroscopeBiasSubmanifoldOffset].get();
}

auto Manifold<IMU>::setGyroscopeBiasSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::GyroscopeBias::StampedVariable::kNumParameters);
  submanifolds_[kGyroscopeBiasSubmanifoldOffset] = std::move(submanifold);
}

auto Manifold<IMU>::setGyroscopeBiasConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::GyroscopeBias::StampedVariable>>(true, constant);
  setGyroscopeBiasSubmanifold(std::move(submanifold));
}

auto Manifold<IMU>::accelerometerBiasSubmanifold() const -> Submanifold* {
  return submanifolds_[kAccelerometerBiasSubmanifoldOffset].get();
}

auto Manifold<IMU>::setAccelerometerBiasSubmanifold(std::unique_ptr<Submanifold>&& submanifold) -> void {
  DCHECK_EQ(submanifold->AmbientSize(), IMU::AccelerometerBias::StampedVariable::kNumParameters);
  submanifolds_[kAccelerometerBiasSubmanifoldOffset] = std::move(submanifold);
}

auto Manifold<IMU>::setAccelerometerBiasConstant(const bool constant) -> void {
  auto submanifold = std::make_unique<Manifold<IMU::AccelerometerBias::StampedVariable>>(true, constant);
  setAccelerometerBiasSubmanifold(std::move(submanifold));
}

}  // namespace hyper::ceres::manifolds

#endif
