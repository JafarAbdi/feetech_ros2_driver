#include <feetech_driver/SMS_STS.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include <cstddef>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>

namespace feetech_driver {

CommunicationProtocol::CommunicationProtocol(std::unique_ptr<SerialPort> serial_port)
    : serial_port_(std::move(serial_port)) {}

Expected<int> CommunicationProtocol::read_word(const uint8_t id, const uint8_t memory_address) {
  std::array<uint8_t, 2> buffer{};
  return read(id, memory_address, &buffer).and_then([&]() -> Expected<int> {
    return from_sts(WordBytes{.low = buffer[0], .high = buffer[1]});
  });
}

Result CommunicationProtocol::read_response(const uint8_t id) {
  if (id == kBroadcastId) {
    return {};
  }
  std::array<uint8_t, 4> buffer{};
  return check_head().and_then([&] { return serial_port_->read(&buffer); }).and_then([&]() -> Result {
    if (buffer[0] != id) {
      return tl::make_unexpected(fmt::format("buffer[0][={}] != id[={}]", buffer[0], id));
    }
    if (buffer[1] != 2) {
      return tl::make_unexpected(fmt::format("buffer[1][={}] != 2", buffer[1]));
    }
    const auto checksum = ~(buffer[0] + buffer[1] + buffer[2]);
    if (static_cast<std::byte>(checksum) != static_cast<std::byte>(buffer[3])) {
      return tl::make_unexpected(fmt::format(
          "CommunicationProtocol::read_response [calculated_checksum={} != checksum={}]", checksum, buffer[3]));
    }
    return {};
  });
}

Result CommunicationProtocol::check_head() {
  std::array<uint8_t, 1> b_dat{};
  std::array<uint8_t, 2> b_buf{};
  const int max_number_of_tries = 10;
  for (int i = 0; i <= max_number_of_tries; i++) {
    if (const auto result = serial_port_->read(&b_dat); !result) {
      return tl::make_unexpected(fmt::format("CommunicationProtocol::check_head -> {}", result.error()));
    }
    b_buf[1] = b_buf[0];
    b_buf[0] = b_dat[0];
    if (b_buf[0] == static_cast<uint8_t>(0xff) && b_buf[1] == static_cast<uint8_t>(0xff)) {
      return {};
    }
  }
  return tl::make_unexpected(fmt::format("Failed to check head after {} tries", max_number_of_tries));
}

Result CommunicationProtocol::ping(int id) {
  std::array<uint8_t, 4> buffer{};
  auto write_result =
      write_buffer(id, 0, kEmptyArray, kInstructionPing).and_then([&] { return check_head(); }).and_then([&] {
        return serial_port_->read(&buffer);
      });
  if (!write_result) {
    return write_result;
  }

  if (buffer[0] != id && id != kBroadcastId) {
    return tl::make_unexpected(fmt::format("buffer[0] != id != BROADCAST_ID", buffer[0], id, kBroadcastId));
  }
  if (buffer[1] != 2) {
    return tl::make_unexpected(fmt::format("buffer[1] != 2", buffer[1]));
  }

  const auto calculated_checksum = ~(buffer[0] + buffer[1] + buffer[2]);
  if (static_cast<std::byte>(calculated_checksum) != static_cast<std::byte>(buffer[3])) {
    return tl::make_unexpected(fmt::format(
        "CommunicationProtocol::ping [calculated_checksum={} != checksum={}]", calculated_checksum, buffer[3]));
  }
  return {};
}

Result CommunicationProtocol::write_position(const uint8_t id, int position, int speed, const int acceleration) {
  std::array<uint8_t, 7> buffer{};
  buffer[0] = acceleration;
  to_sts(&buffer[1], &buffer[2], encode_signed_value(position));
  to_sts(&buffer[3], &buffer[4], 0);
  to_sts(&buffer[5], &buffer[6], encode_signed_value(speed));
  return write(id, SMS_STS_ACC, buffer);
}

Expected<int> CommunicationProtocol::read_position(const uint8_t id) {
  return read_word(id, SMS_STS_PRESENT_POSITION_L).and_then([](auto position) -> Expected<int> {
    return encode_signed_value(position);
  });
}

Expected<int> CommunicationProtocol::read_speed(const uint8_t id) {
  return read_word(id, SMS_STS_PRESENT_SPEED_L).and_then([](auto speed) -> Expected<int> {
    return encode_signed_value(speed);
  });
}

Result CommunicationProtocol::set_torque(const uint8_t id, const bool enable) {
  return write(id, SMS_STS_TORQUE_ENABLE, std::experimental::make_array(static_cast<uint8_t>(enable ? 1 : 0)));
}

Result CommunicationProtocol::calbration_offset(const uint8_t id) {
  return write(id, SMS_STS_TORQUE_ENABLE, std::experimental::make_array(uint8_t{128}));
}

Result CommunicationProtocol::set_maximum_angle_limit(const uint8_t id, const int angle) {
  std::array<uint8_t, 2> buf{};
  if (angle < 0) {
    to_sts(buf.data(), &buf[1], 0);
  } else if (angle > 4095) {
    to_sts(buf.data(), &buf[1], 4095);
  } else {
    to_sts(buf.data(), &buf[1], angle);
  }
  return write(id, SMS_STS_MAX_ANGLE_LIMIT_L, buf);
}

Result CommunicationProtocol::set_minimum_angle_limit(const uint8_t id, const int angle) {
  std::array<uint8_t, 2> buf{};
  if (angle < 0) {
    to_sts(buf.data(), &buf[1], 0);
  } else if (angle > 4095) {
    to_sts(buf.data(), &buf[1], 4095);
  } else {
    to_sts(buf.data(), &buf[1], angle);
  }
  return write(id, SMS_STS_MIN_ANGLE_LIMIT_L, buf);
}

Result CommunicationProtocol::lock_eprom(const uint8_t id) {
  return write(id, SMS_STS_LOCK, std::experimental::make_array(uint8_t{1}));
}

Result CommunicationProtocol::unlock_eprom(const uint8_t id) {
  return write(id, SMS_STS_LOCK, std::experimental::make_array(uint8_t{0}));
}

Result CommunicationProtocol::reg_write_action(const uint8_t id) {
  return write_buffer(id, 0, kEmptyArray, kInstructionRegAction).and_then([&] { return read_response(id); });
}

Expected<int> CommunicationProtocol::read_model_number(uint8_t id) { return read_word(id, SMS_STS_MODEL_L); }
}  // namespace feetech_driver
