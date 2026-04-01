#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <type_traits>
#include <vector>

inline constexpr std::size_t max_serialized_elements = 1 << 26; // ~67M elements

template <typename T>
void serialize_vector(const std::filesystem::path &fpath, const std::vector<T> &vec) {
  static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable.");

  std::ofstream file{fpath, std::ios::binary};
  if (!file)
    throw std::runtime_error("Unable to open file for writing: " + fpath.string());

  auto count = vec.size();

  file.write(reinterpret_cast<const char *>(&count), sizeof(count));
  file.write(reinterpret_cast<const char *>(vec.data()), sizeof(T) * count);

  if (!file)
    throw std::runtime_error("Failed to write vector data: " + fpath.string());
}

template <typename T> std::vector<T> deserialize_vector(const std::filesystem::path &fpath) {
  static_assert(std::is_trivially_copyable_v<T>, "Type must be trivially copyable.");

  std::ifstream file{fpath, std::ios::binary};
  if (!file)
    throw std::runtime_error("Unable to open file for reading: " + fpath.string());

  std::size_t count{0};
  if (!file.read(reinterpret_cast<char *>(&count), sizeof(count)))
    throw std::runtime_error("Failed to read element count: " + fpath.string());

  if (count > max_serialized_elements)
    throw std::runtime_error("Element count exceeds safety limit: " + std::to_string(count));

  std::vector<T> out(count);

  if (!file.read(reinterpret_cast<char *>(out.data()), sizeof(T) * count))
    throw std::runtime_error("Failed to read vector data: " + fpath.string());

  return out;
}

