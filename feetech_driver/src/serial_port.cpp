#include <spdlog/spdlog.h>

#include <feetech_hardware_interface/serial_port.hpp>
#include <tl_expected/expected.hpp>

namespace feetech_hardware_interface {

Expected<LibSerial::BaudRate> to_baudrate(const std::size_t baud) noexcept {
  using LibSerial::BaudRate;
  switch (baud) {
    case 50:
      return BaudRate::BAUD_50;
    case 75:
      return BaudRate::BAUD_75;
    case 110:
      return BaudRate::BAUD_110;
    case 134:
      return BaudRate::BAUD_134;
    case 150:
      return BaudRate::BAUD_150;
    case 200:
      return BaudRate::BAUD_200;
    case 300:
      return BaudRate::BAUD_300;
    case 600:
      return BaudRate::BAUD_600;
    case 1'200:
      return BaudRate::BAUD_1200;
    case 1'800:
      return BaudRate::BAUD_1800;
    case 2'400:
      return BaudRate::BAUD_2400;
    case 4'800:
      return BaudRate::BAUD_4800;
    case 9'600:
      return BaudRate::BAUD_9600;
    case 19'200:
      return BaudRate::BAUD_19200;
    case 38'400:
      return BaudRate::BAUD_38400;
    case 57'600:
      return BaudRate::BAUD_57600;
    case 115'200:
      return BaudRate::BAUD_115200;
    case 230'400:
      return BaudRate::BAUD_230400;
#ifdef __linux__
    case 460'800:
      return BaudRate::BAUD_460800;
    case 500'000:
      return BaudRate::BAUD_500000;
    case 576'000:
      return BaudRate::BAUD_576000;
    case 921'600:
      return BaudRate::BAUD_921600;
    case 1'000'000:
      return BaudRate::BAUD_1000000;
    case 1'152'000:
      return BaudRate::BAUD_1152000;
    case 1'500'000:
      return BaudRate::BAUD_1500000;
#if __MAX_BAUD > B2000000
    case 2'000'000:
      return BaudRate::BAUD_2000000;
    case 2'500'000:
      return BaudRate::BAUD_2500000;
    case 3'000'000:
      return BaudRate::BAUD_3000000;
    case 3'500'000:
      return BaudRate::BAUD_3500000;
    case 4'000'000:
      return BaudRate::BAUD_4000000;
#endif /* __MAX_BAUD */
#endif /* __linux__ */
  }

  return tl::make_unexpected(fmt::format("Invalid baud rate: [{}]", baud));
}

SerialPort::SerialPort(const std::string& dev) : dev_(dev) { spdlog::info("Connecting to port: {}", dev); }

SerialPort::~SerialPort() { close(); }

Result SerialPort::configure(const LibSerial::BaudRate baud_rate) {
  if (auto result = open(); !result) {
    return result;
  }

  try {
    port_.SetBaudRate(baud_rate);
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(fmt::format("Configuring the serial port failed: [{}]", e.what()));
  }
  return {};
}

Result SerialPort::open() {
  try {
    if (!port_.IsOpen()) {
      port_.Open(dev_);
    }
  } catch (const LibSerial::OpenFailed& e) {
    return tl::make_unexpected(fmt::format("Open [{}]: {}", dev_.c_str(), e.what()));
  }

  return {};
}

Result SerialPort::close() {
  try {
    port_.Close();
  } catch (const LibSerial::AlreadyOpen& e) {
    return tl::make_unexpected(fmt::format("close [{}]: {}", dev_.c_str(), e.what()));
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(fmt::format("close [{}]: {}", dev_.c_str(), e.what()));
  }
  return {};
}

Result SerialPort::check_port() const noexcept {
  if (!port_.IsOpen()) {
    return tl::make_unexpected(fmt::format("Port [{}] is not open", dev_));
  }

  return {};
}

Result SerialPort::flashInputBuffer() noexcept {
  if (auto result = check_port(); !result) {
    return result;
  }

  try {
    port_.FlushInputBuffer();
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(e.what());
  }

  return {};
}

Result SerialPort::flashOutputBuffer() noexcept {
  if (auto result = check_port(); !result) {
    return result;
  }

  try {
    port_.FlushOutputBuffer();
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(e.what());
  }

  return {};
}
}  // namespace feetech_hardware_interface
