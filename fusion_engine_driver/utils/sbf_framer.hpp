#ifndef FUSION_ENGINE_DRIVER__UTILS__SBF_FRAMER_HPP_
#define FUSION_ENGINE_DRIVER__UTILS__SBF_FRAMER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "sbf_utils.hpp"  // crc16()

/**
 * @brief Reassembles a continuous SBF byte stream into complete, CRC-validated
 * blocks.
 *
 * The Septentrio (mosaic) SBF stream is delivered here in arbitrary chunks
 * (FusionEngine INPUT_DATA_WRAPPER payloads) that do NOT align to SBF block
 * boundaries: a chunk may start mid-block, carry a partial block, or carry
 * several. This framer buffers bytes across pushes, syncs on the '$@' preamble,
 * waits for the full block, verifies the CRC, and hands each complete block to
 * the callback. Single-threaded: feed it from the one thread that reads the
 * socket.
 */
class SbfFramer
{
public:
  /// Called once per complete, CRC-valid block with (block start, total length).
  using BlockCallback = std::function<void(const uint8_t *, size_t)>;

  /// Append `len` bytes of SBF stream and dispatch any newly-complete blocks.
  void push(const uint8_t * data, size_t len, const BlockCallback & cb)
  {
    buffer_.insert(buffer_.end(), data, data + len);
    parse(cb);
    if (buffer_.size() > kMaxBufferBytes) {
      buffer_.clear();  // runaway guard: never seen a valid block
    }
  }

private:
  static constexpr size_t kHeaderLen = 8;         // sync(2) crc(2) id(2) length(2)
  static constexpr uint16_t kMaxBlockLen = 4096;  // SBF blocks are small
  static constexpr size_t kMaxBufferBytes = 65536;

  void parse(const BlockCallback & cb)
  {
    size_t i = 0;
    const size_t n = buffer_.size();
    while (i + kHeaderLen <= n) {
      if (!(buffer_[i] == 0x24 && buffer_[i + 1] == 0x40)) {  // '$' '@'
        ++i;
        continue;
      }
      const uint16_t length = buffer_[i + 6] | (buffer_[i + 7] << 8);
      if (length < kHeaderLen || (length % 4) != 0 || length > kMaxBlockLen) {
        ++i;  // false sync -> resync one byte on
        continue;
      }
      if (i + length > n) {
        break;  // block not fully buffered yet; wait for more bytes
      }
      const uint16_t crc_read = buffer_[i + 2] | (buffer_[i + 3] << 8);
      const uint16_t crc_calc = crc16(&buffer_[i + 4], length - 4, 0);
      if (crc_calc == crc_read) {
        cb(&buffer_[i], length);
        i += length;
      } else {
        ++i;  // bad CRC -> false sync, resync
      }
    }
    if (i > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + i);
    }
  }

  std::vector<uint8_t> buffer_;
};

#endif  // FUSION_ENGINE_DRIVER__UTILS__SBF_FRAMER_HPP_
