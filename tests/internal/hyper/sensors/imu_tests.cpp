/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <string>

#include <gtest/gtest.h>

#include "hyper/sensors/imu.hpp"
#include "hyper/utils.hpp"

namespace hyper::sensors::tests {

class IMUTests : public testing::Test {
 protected:
  // Constants.
  static constexpr auto kFilePath = __FILE__;
  static constexpr auto kInputFileName = "imu.yaml";
  static constexpr auto kOutputFileName = "imu_out.yaml";

  // Definitions.
  using Path = std::filesystem::path;
  using IMU = sensors::IMU;

  /// Setup.
  auto SetUp() -> void final {
    input_ = Path{kFilePath}.parent_path() /= kInputFileName;
    YAML::LoadFile(input_) >> imu_;
  }

  Path input_;
  IMU imu_;
};

TEST_F(IMUTests, ReadWrite) {
  YAML::Emitter emitter;
  const auto output = Path{kFilePath}.parent_path() /= kOutputFileName;
  std::ofstream{output} << (emitter << imu_).c_str();
  EXPECT_TRUE(internal::CompareFiles(input_, output));
  std::remove(output.c_str());
}

}  // namespace hyper::sensors::tests
