#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace cnpy {

  struct NpyArray
  {
    std::vector < char > data;
    std::vector < size_t > shape;
    size_t word_size = 0;
    bool fortran_order = false;

    size_t num_bytes() const {return data.size();}
  };

  using npz_t = std::unordered_map < std::string, NpyArray >;

  npz_t npz_load_all(const std::string & file_name);

}  // namespace cnpy
