/// This file is subject to the terms and conditions defined in
/// the 'LICENSE' file, which is part of this repository.

#include <filesystem>
#include <fstream>

// Definitions.
using Path = std::filesystem::path;

namespace hyper::sensors::tests::internal {

/// Compares to files.
/// \param p1 Path to first file.
/// \param p2 Path to second file.
/// \return True if files are identical.
static auto CompareFiles(const Path& p1, const Path& p2) -> bool {
  std::ifstream f1(p1, std::ifstream::binary | std::ifstream::ate);
  std::ifstream f2(p2, std::ifstream::binary | std::ifstream::ate);

  if (f1.fail() || f2.fail()) {
    return false;
  }

  if (f1.tellg() != f2.tellg()) {
    return false;
  }

  f1.seekg(0, std::ifstream::beg);
  f2.seekg(0, std::ifstream::beg);
  return std::equal(std::istreambuf_iterator<char>(f1.rdbuf()), std::istreambuf_iterator<char>(), std::istreambuf_iterator<char>(f2.rdbuf()));
}

}  // namespace hyper::sensors::tests::internal
