/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <string>

#include <gtest/gtest.h>

#include "hyper/sensors/sensor.hpp"
#include "hyper/utils.hpp"

namespace hyper::sensors::tests {

class SensorTests : public testing::Test {
 protected:
  // Constants.
  static constexpr auto kFilePath = __FILE__;
  static constexpr auto kInputFileName = "sensor.yaml";
  static constexpr auto kOutputFileName = "sensor_out.yaml";

  // Definitions.
  using Path = std::filesystem::path;

  using Scalar = double;
  using Sensor = sensors::Sensor;

  /// Setup.
  auto SetUp() -> void final {
    input_ = Path{kFilePath}.parent_path() /= kInputFileName;
    YAML::LoadFile(input_) >> sensor_;
  }

  Path input_;
  Sensor sensor_;
};

TEST_F(SensorTests, ReadWrite) {
  YAML::Emitter emitter;
  const auto output = Path{kFilePath}.parent_path() /= kOutputFileName;
  std::ofstream{output} << (emitter << sensor_).c_str();
  EXPECT_TRUE(internal::CompareFiles(input_, output));
  std::remove(output.c_str());
}

}  // namespace hyper::sensors::tests
