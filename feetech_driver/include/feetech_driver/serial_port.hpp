#pragma once

#include <fmt/core.h>
#include <libserial/SerialPort.h>

#include <chrono>
#include <cstring>
#include <feetech_driver/common.hpp>
#include <range/v3/all.hpp>
#include <string>

namespace feetech_driver {

class SerialPort {
 public:
  explicit SerialPort(const std::string& /*dev*/);
  ~SerialPort();
  Result configure(LibSerial::BaudRate baud_rate = LibSerial::BaudRate::BAUD_1000000);
  Result open();
  Result close();
  Result flashInputBuffer() noexcept;
  Result flashOutputBuffer() noexcept;

  Result read_byte(uint8_t* byte) {
    try {
      port_.ReadByte(*byte, static_cast<std::size_t>(timeout_.count()));
    } catch (const LibSerial::ReadTimeout& e) {
      return tl::make_unexpected(fmt::format("SerialPort::read_byte [{}]", e.what()));
    }

    return {};
  }

  Result read_exact(uint8_t* dst, size_t n) {
    try {
      std::string s;
      s.resize(n);
      // Read exactly n bytes (or throw ReadTimeout)
      port_.Read(s, n, static_cast<std::size_t>(timeout_.count()));
      std::memcpy(dst, s.data(), n);
      return {};
    } catch (const LibSerial::ReadTimeout& e) {
      return tl::make_unexpected(fmt::format("SerialPort::read_exact [{}]", e.what()));
    } catch (const std::runtime_error& e) {
      return tl::make_unexpected(fmt::format("SerialPort::read_exact [{}]", e.what()));
    }
  }

  template <std::size_t N>
  Result read(std::array<uint8_t, N>* buffer) {
    return check_port().and_then([&]() { return read_exact(buffer->data(), N); });
  }

  template <std::size_t N>
  Result write(const std::array<uint8_t, N>& buffer) {
    return check_port().and_then([&]() -> Result {
      try {
        port_.Write(std::string(buffer.begin(), buffer.end()));
      } catch (const std::runtime_error& e) {
        return tl::make_unexpected(fmt::format("SerialPort::write [{}]", e.what()));
      }
      return {};
    });
  }

 private:
  [[nodiscard]] Result check_port() const noexcept;
  std::string dev_;
  std::chrono::milliseconds timeout_ = std::chrono::milliseconds(5);
  LibSerial::SerialPort port_;
};
}  // namespace feetech_driver
