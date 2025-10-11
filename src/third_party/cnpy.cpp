#include "emcl2/detail/cnpy.h"

#include <zlib.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

namespace cnpy
{
namespace
{

uint16_t read_le16(const char * ptr)
{
  uint16_t value;
  std::memcpy(&value, ptr, sizeof(uint16_t));
  return value;
}

uint32_t read_le32(const char * ptr)
{
  uint32_t value;
  std::memcpy(&value, ptr, sizeof(uint32_t));
  return value;
}

std::vector<char> decompress_deflate(const char * src, size_t comp_size, size_t uncomp_size)
{
  if (uncomp_size == 0U) {
    return {};
  }

  std::vector<char> dst(uncomp_size);

  z_stream stream{};
  stream.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(src));
  stream.avail_in = static_cast<uInt>(comp_size);
  stream.next_out = reinterpret_cast<Bytef *>(dst.data());
  stream.avail_out = static_cast<uInt>(uncomp_size);

  if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) {
    throw std::runtime_error("inflateInit2 failed");
  }

  const int ret = inflate(&stream, Z_FINISH);
  if (ret != Z_STREAM_END) {
    inflateEnd(&stream);
    throw std::runtime_error("inflate failed");
  }

  if (inflateEnd(&stream) != Z_OK) {
    throw std::runtime_error("inflateEnd failed");
  }

  if (stream.total_out != uncomp_size) {
    throw std::runtime_error("Decompressed size mismatch");
  }

  return dst;
}

std::string trim_copy(std::string s)
{
  s.erase(
    s.begin(), std::find_if(
      s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
      }));
  s.erase(
    std::find_if(
      s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
      }).base(),
    s.end());
  return s;
}

std::string find_token(const std::string & header, const std::string & key)
{
  auto pos = header.find(key);
  if (pos == std::string::npos) {
    return {};
  }
  pos = header.find(':', pos);
  if (pos == std::string::npos) {
    return {};
  }
  ++pos;
  size_t end = header.find(',', pos);
  if (end == std::string::npos) {
    end = header.find('}', pos);
  }
  if (end == std::string::npos) {
    end = header.size();
  }
  std::string token = header.substr(pos, end - pos);
  token = trim_copy(token);
  if (!token.empty() && token.front() == '\'' && token.back() == '\'') {
    token = token.substr(1, token.size() - 2);
  }
  return token;
}

void parse_npy(const std::vector<char> & raw, NpyArray & array)
{
  if (raw.size() < 10U) {
    throw std::runtime_error("Invalid npy file: truncated header");
  }

  const char magic[] = {char(0x93), 'N', 'U', 'M', 'P', 'Y'};
  if (!std::equal(std::begin(magic), std::end(magic), raw.begin())) {
    throw std::runtime_error("Invalid npy magic header");
  }

  const uint8_t major = static_cast<uint8_t>(raw[6]);
  size_t header_len = 0;
  size_t header_offset = 0;
  if (major == 1U) {
    header_len = read_le16(raw.data() + 8);
    header_offset = 10;
  } else if (major == 2U) {
    header_len = read_le32(raw.data() + 8);
    header_offset = 12;
  } else {
    throw std::runtime_error("Unsupported npy version");
  }
  if (header_offset + header_len > raw.size()) {
    throw std::runtime_error("Invalid npy header length");
  }

  const std::string header(raw.begin() + header_offset, raw.begin() + header_offset + header_len);

  const std::string descr = find_token(header, "'descr'");
  if (descr.size() < 2U) {
    throw std::runtime_error("Failed to parse dtype");
  }
  size_t num_start = descr.size();
  while (num_start > 0U && std::isdigit(static_cast<unsigned char>(descr[num_start - 1U]))) {
    --num_start;
  }
  if (num_start == descr.size()) {
    throw std::runtime_error("Unable to determine dtype size");
  }
  array.word_size = static_cast<size_t>(std::stoul(descr.substr(num_start)));

  const std::string fortran = find_token(header, "'fortran_order'");
  array.fortran_order = (fortran == "True");

  const std::size_t shape_pos = header.find("'shape'");
  if (shape_pos != std::string::npos) {
    const std::size_t open = header.find('(', shape_pos);
    const std::size_t close = header.find(')', open);
    if (open != std::string::npos && close != std::string::npos && close > open + 1U) {
      const std::string shape_token = header.substr(open + 1U, close - open - 1U);
      size_t start = 0;
      while (start < shape_token.size()) {
        const size_t comma = shape_token.find(',', start);
        const size_t end = (comma == std::string::npos) ? shape_token.size() : comma;
        const std::string dim_str = trim_copy(shape_token.substr(start, end - start));
        if (!dim_str.empty()) {
          array.shape.push_back(static_cast<size_t>(std::stoll(dim_str)));
        }
        if (comma == std::string::npos) {
          break;
        }
        start = comma + 1U;
      }
    }
  }

  const size_t data_offset = header_offset + header_len;
  array.data.assign(raw.begin() + data_offset, raw.end());
}

}  // namespace

npz_t npz_load_all(const std::string & file_name)
{
  std::ifstream file(file_name, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Failed to open npz file: " + file_name);
  }
  file.seekg(0, std::ios::end);
  const std::streamoff file_size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<char> buffer(static_cast<size_t>(file_size));
  if (!buffer.empty()) {
    file.read(buffer.data(), file_size);
  }
  file.close();

  // Locate end of central directory record.
  size_t eocd_pos = std::string::npos;
  const uint32_t eocd_signature = 0x06054b50;
  const size_t start_pos = buffer.size() >= 22U ? buffer.size() - 22U : 0U;
  for (size_t pos = start_pos + 1U; pos-- > 0U; ) {
    if (pos + 4U <= buffer.size() && read_le32(buffer.data() + pos) == eocd_signature) {
      eocd_pos = pos;
      break;
    }
    if (pos == 0U) {
      break;
    }
  }
  if (eocd_pos == std::string::npos) {
    throw std::runtime_error("Failed to locate end of central directory in npz file");
  }

  const char * eocd = buffer.data() + eocd_pos;
  const uint16_t total_entries = read_le16(eocd + 10);
  const uint32_t central_dir_offset = read_le32(eocd + 16);

  npz_t result;
  size_t ptr = central_dir_offset;
  const uint32_t cdfh_signature = 0x02014b50;
  const uint32_t lfh_signature = 0x04034b50;

  for (uint16_t entry = 0; entry < total_entries; ++entry) {
    if (ptr + 46U > buffer.size()) {
      throw std::runtime_error("Central directory truncated");
    }
    if (read_le32(buffer.data() + ptr) != cdfh_signature) {
      break;
    }

    const uint16_t comp_method = read_le16(buffer.data() + ptr + 10);
    const uint32_t comp_size = read_le32(buffer.data() + ptr + 20);
    const uint32_t uncomp_size = read_le32(buffer.data() + ptr + 24);
    const uint16_t name_len = read_le16(buffer.data() + ptr + 28);
    const uint16_t extra_len = read_le16(buffer.data() + ptr + 30);
    const uint16_t comment_len = read_le16(buffer.data() + ptr + 32);
    const uint32_t local_header_offset = read_le32(buffer.data() + ptr + 42);

    if (ptr + 46U + name_len > buffer.size()) {
      throw std::runtime_error("Central directory name truncated");
    }
    std::string name(buffer.data() + ptr + 46, name_len);

    ptr += 46U + static_cast<size_t>(name_len) + static_cast<size_t>(extra_len) +
      static_cast<size_t>(comment_len);

    if (name.size() < 4U || name.substr(name.size() - 4U) != ".npy") {
      continue;
    }

    if (local_header_offset + 30U > buffer.size()) {
      throw std::runtime_error("Local header truncated");
    }
    if (read_le32(buffer.data() + local_header_offset) != lfh_signature) {
      throw std::runtime_error("Invalid local file header signature");
    }
    const uint16_t local_name_len = read_le16(buffer.data() + local_header_offset + 26);
    const uint16_t local_extra_len = read_le16(buffer.data() + local_header_offset + 28);

    const size_t data_offset = static_cast<size_t>(local_header_offset) + 30U +
      static_cast<size_t>(local_name_len) + static_cast<size_t>(local_extra_len);
    if (data_offset + comp_size > buffer.size()) {
      throw std::runtime_error("Compressed data truncated");
    }

    const char * comp_ptr = buffer.data() + data_offset;
    std::vector<char> raw_data;
    if (comp_method == 0U) {
      raw_data.assign(comp_ptr, comp_ptr + comp_size);
    } else if (comp_method == 8U) {
      raw_data = decompress_deflate(comp_ptr, comp_size, uncomp_size);
    } else {
      throw std::runtime_error("Unsupported compression method in npz");
    }

    NpyArray array;
    parse_npy(raw_data, array);
    result.emplace(std::move(name), std::move(array));
  }

  return result;
}

}  // namespace cnpy
