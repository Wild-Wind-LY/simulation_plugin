#pragma once

// Self-contained SHA-256 used for content-addressed cache keys and asset
// integrity digests, so the plugin does not depend on an external crypto
// library. Header-only + inline so it can be shared across translation units.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace simulation {

class Sha256 {
public:
  Sha256() = default;

  void update(const void* bytes, size_t len) {
    const auto* data = static_cast<const unsigned char*>(bytes);
    total_ += len;
    while (len > 0) {
      const size_t take = std::min(len, size_t{64} - buffer_len_);
      std::memcpy(buffer_.data() + buffer_len_, data, take);
      buffer_len_ += take;
      data += take;
      len -= take;
      if (buffer_len_ == 64) {
        transform(buffer_.data());
        buffer_len_ = 0;
      }
    }
  }

  void update(const std::string& s) { update(s.data(), s.size()); }

  std::string hex() {
    const uint64_t bit_len = total_ * 8;
    const unsigned char one = 0x80;
    update(&one, 1);
    const unsigned char zero = 0x00;
    while (buffer_len_ != 56) update(&zero, 1);
    unsigned char length_bytes[8];
    for (int i = 0; i < 8; ++i)
      length_bytes[i] = static_cast<unsigned char>((bit_len >> (56 - i * 8)) & 0xff);
    update(length_bytes, 8);

    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(64);
    for (uint32_t word : state_) {
      for (int shift = 28; shift >= 0; shift -= 4)
        out.push_back(digits[(word >> shift) & 0xf]);
    }
    return out;
  }

private:
  static uint32_t ror(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

  void transform(const unsigned char* p) {
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2};

    uint32_t w[64];
    for (int i = 0; i < 16; ++i)
      w[i] = (static_cast<uint32_t>(p[i * 4]) << 24) | (static_cast<uint32_t>(p[i * 4 + 1]) << 16)
             | (static_cast<uint32_t>(p[i * 4 + 2]) << 8) | static_cast<uint32_t>(p[i * 4 + 3]);
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = ror(w[i - 15], 7) ^ ror(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const uint32_t s1 = ror(w[i - 2], 17) ^ ror(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
    uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t s1 = ror(e, 6) ^ ror(e, 11) ^ ror(e, 25);
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = h + s1 + ch + k[i] + w[i];
      const uint32_t s0 = ror(a, 2) ^ ror(a, 13) ^ ror(a, 22);
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = s0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<uint32_t, 8> state_{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::array<unsigned char, 64> buffer_{};
  size_t buffer_len_ = 0;
  uint64_t total_ = 0;
};

inline std::string sha256_string(const std::string& value) {
  Sha256 hasher;
  hasher.update(value);
  return hasher.hex();
}

inline void sha256_feed_file(Sha256& hasher, const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("failed to open for hashing: " + path.string());
  std::array<char, 1 << 16> chunk{};
  while (input) {
    input.read(chunk.data(), chunk.size());
    const std::streamsize got = input.gcount();
    if (got > 0) hasher.update(chunk.data(), static_cast<size_t>(got));
  }
}

inline std::string sha256_file(const std::filesystem::path& path) {
  Sha256 hasher;
  sha256_feed_file(hasher, path);
  return hasher.hex();
}

// Deterministic digest over every regular file under `dir` (relative path +
// content), so tampering with any packaged asset (mesh, texture, model) is
// detectable, not just the top-level model file.
inline std::string sha256_directory(const std::filesystem::path& dir) {
  namespace fs = std::filesystem;
  std::vector<fs::path> files;
  for (auto it = fs::recursive_directory_iterator(
           dir, fs::directory_options::skip_permission_denied);
       it != fs::recursive_directory_iterator(); ++it) {
    if (it->is_regular_file()) files.push_back(fs::relative(it->path(), dir));
  }
  std::sort(files.begin(), files.end(), [](const fs::path& a, const fs::path& b) {
    return a.generic_string() < b.generic_string();
  });

  Sha256 hasher;
  for (const auto& rel : files) {
    const std::string name = rel.generic_string();
    hasher.update(name);
    const unsigned char sep = 0x00;
    hasher.update(&sep, 1);
    sha256_feed_file(hasher, dir / rel);
    hasher.update(&sep, 1);
  }
  return hasher.hex();
}

}  // namespace simulation
