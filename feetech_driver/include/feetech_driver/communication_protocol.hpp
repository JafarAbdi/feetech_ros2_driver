#pragma once

#include <feetech_hardware_interface/SMS_STS.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>
#include <sys/types.h>

#include <experimental/array>
#include <feetech_hardware_interface/serial_port.hpp>
#include <numeric>

namespace feetech_hardware_interface {

enum class OperationMode {
  kPosition,
  kSpeed,
  kOpenLoop,  // PWM
  kStepServo,
};

enum class Mode {
  kSequential,
  kSynchronous,
  kAsynchronous,
};

class CommunicationProtocol {
 public:
  explicit CommunicationProtocol(std::unique_ptr<SerialPort> /*serial_port*/);

  Result ping(int id);

  Result write_position(uint8_t id, int position, int speed, int acceleration);

  // From User Manual: The real-time performance of this command is higher. A SYNC WRITE command can modify the contents
  // of control tables of multiple servos at one time, while the REG WRITE+ACTION command is done step by step.
  template <std::size_t N>
  Result sync_write(const std::vector<uint8_t>& ids,
                    const uint8_t memory_address,
                    const std::vector<std::array<uint8_t, N>>& parameters) {
    if (ids.size() != parameters.size()) {
      return tl::make_unexpected(
          fmt::format("IDs[={}] and parameters[={}] must have the same size", ids.size(), parameters.size()));
    }

    std::array<uint8_t, 7> buffer{
        0xff,
        0xff,
        kBroadcastId,
        static_cast<uint8_t>((N + 1) * ids.size() + 4),  // Message length = #parameters * (#parameters + 1 (id)) + 4
        kInstructionSyncWrite,
        memory_address,
        N};
    uint8_t checksum = buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6];

    return serial_port_->write(buffer)
        .and_then([&]() -> Result {
          for (size_t i = 0; i < ids.size(); ++i) {
            auto result = serial_port_->write(std::experimental::make_array(ids[i])).and_then([&] {
              return serial_port_->write(parameters[i]);
            });
            if (!result) {
              return tl::make_unexpected(
                  fmt::format("CommunicationProtocol::sync_write with [id={}, parameters={}]: [{}]",
                              ids[i],
                              parameters[i],
                              result.error()));
            }
            checksum += ids[i] + sum_bytes(parameters[i]);
          }
          return {};
        })
        .and_then([&] { return serial_port_->write(std::experimental::make_array(static_cast<uint8_t>(~checksum))); });
  }

  /// TODO: Should I create a struct??
  /// TOOD: Should I make speed/acceleration optional?
  Result sync_write_position(const std::vector<uint8_t>& ids,
                             const std::vector<int>& position,
                             const std::vector<int>& speed,
                             const std::vector<int>& acceleration) {
    if (ids.size() != position.size() || ids.size() != speed.size() || ids.size() != acceleration.size()) {
      return tl::make_unexpected(
          fmt::format("Sizes of IDs, position, speed, and acceleration must be the same - ids[{}], position[{}], "
                      "speed[{}], acceleration[{}]",
                      ids.size(),
                      position.size(),
                      speed.size(),
                      acceleration.size()));
    }
    std::vector<std::array<uint8_t, 7>> buffer;  // TODO: Rename to parameters?
    buffer.resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      buffer[i][0] = acceleration[i];
      to_sts(&buffer[i][1], &buffer[i][2], encode_signed_value(position[i]));
      to_sts(&buffer[i][3], &buffer[i][4], 0);  // Time
      to_sts(&buffer[i][5], &buffer[i][6], speed[i]);
    }
    return sync_write(ids, SMS_STS_ACC, buffer);
  }

  Result reg_write_position(const uint8_t id, const int position, const int speed, const int acceleration) {
    std::array<uint8_t, 7> buffer{};
    buffer[0] = acceleration;
    to_sts(&buffer[1], &buffer[2], encode_signed_value(position));
    to_sts(&buffer[3], &buffer[4], 0);  // Time
    to_sts(&buffer[5], &buffer[6], encode_signed_value(speed));
    return reg_write(id, SMS_STS_ACC, buffer);
  }

  template <std::size_t N>
  Result reg_write(const uint8_t id, const uint8_t memory_address, const std::array<uint8_t, N>& parameters) {
    return write_buffer(id, memory_address, parameters, kInstructionRegWrite).and_then([&] {
      return read_response(id);
    });
  }

  Result set_torque(uint8_t id, bool enable);
  Result calbration_offset(uint8_t id);

  Result set_maximum_angle_limit(uint8_t id, int angle);
  Result set_minimum_angle_limit(uint8_t id, int angle);
  Result set_mode(const uint8_t id, const OperationMode mode) {
    return write(id, SMS_STS_MODE, std::experimental::make_array(static_cast<uint8_t>(mode)));
  }

  /// Asynchronous write execution command
  /// @param id The ID of the servo
  Result reg_write_action(uint8_t id = kBroadcastId);

  Expected<int> read_word(uint8_t /*id*/, uint8_t /*memory_address*/);
  Expected<int> read_position(uint8_t id);
  Expected<int> read_speed(uint8_t id);
  Expected<int> read_model_number(uint8_t id);

  Result lock_eprom(uint8_t id);
  Result unlock_eprom(uint8_t id);

  /// TODO: Should we pass the length and this returns Expected<std::vector<std::vector<uint8_t>>>?
  /// Sync Read
  template <std::size_t N>
  Result sync_read(const std::vector<uint8_t>& ids,
                   const uint8_t memory_address,
                   std::vector<std::array<uint8_t, N>>* data) {
    std::array<uint8_t, 7> buffer{0,
                                  0,
                                  kBroadcastId,
                                  static_cast<uint8_t>(ids.size() + 4),  // Message length = #servos(=ids) + 4
                                  kInstructionSyncRead,
                                  memory_address,
                                  N};
    uint8_t request_checksum = sum_bytes(buffer);

    // Set these two after calculating the checksum
    buffer[0] = 0xff;
    buffer[1] = 0xff;

    auto request_result = serial_port_->write(buffer).and_then([&]() -> Result {
      for (const uint8_t id : ids) {
        if (auto result = serial_port_->write(std::experimental::make_array(id)); !result) {
          return tl::make_unexpected(fmt::format("CommunicationProtocol::sync_read [{}]", result.error()));
        }
        request_checksum += id;
      }
      return serial_port_->write(std::experimental::make_array(static_cast<uint8_t>(~request_checksum)));
    });

    if (!request_result) {
      return request_result;
    }

    data->resize(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {
      std::array<uint8_t, 3> response_buffer{};  // ID, Effective Data length, Working status
      uint8_t checksum{};
      auto read_result = check_head()
                             .and_then([&] { return serial_port_->read(&response_buffer); })
                             .and_then([&] { return serial_port_->read(&data->at(i)); })
                             .and_then([&] { return serial_port_->read_byte(&checksum); });
      if (!read_result) {
        return tl::make_unexpected(fmt::format("CommunicationProtocol::sync_read [{}]", read_result.error()));
      }
      const auto calculated_checksum = ~(sum_bytes(response_buffer) + sum_bytes(data->at(i)));
      if (static_cast<std::byte>(calculated_checksum) != static_cast<std::byte>(checksum)) {
        return tl::make_unexpected(fmt::format(
            "CommunicationProtocol::sync_read [calculated_checksum={}, checksum={}]", calculated_checksum, checksum));
      }
    }
    return {};
  }

  /// Normal read command
  /// @tparam N The size of the buffer
  /// @param id The ID of the servo
  /// @param memory_address The memory address to read from
  /// @param data The buffer to store the data
  template <std::size_t N>
  Result read(const uint8_t id, const uint8_t memory_address, std::array<uint8_t, N>* data) {
    auto write_result = write_buffer(id, memory_address, std::array<uint8_t, 1>{N}, kInstructionRead).and_then([&] {
      return check_head();
    });

    if (!write_result) {
      return tl::make_unexpected(fmt::format("CommunicationProtocol::read -> {}", write_result.error()));
    }

    // TODO: Refactor into a function
    std::array<uint8_t, 3> b_buf{};
    uint8_t checksum{};
    auto read_result = serial_port_->read(&b_buf).and_then([&] { return serial_port_->read(data); }).and_then([&] {
      return serial_port_->read_byte(&checksum);
    });

    if (!read_result) {
      return read_result;
    }
    const auto calculated_checksum = ~(sum_bytes(b_buf) + sum_bytes(*data));

    if (static_cast<std::byte>(calculated_checksum) != static_cast<std::byte>(checksum)) {
      return tl::make_unexpected(fmt::format(
          "CommunicationProtocol::read [calculated_checksum={}, checksum={}]", calculated_checksum, checksum));
    }
    return {};
  }

  /// Normal write command
  /// @tparam N The size of the buffer
  /// @param id The ID of the servo
  /// @param memory_address The memory address to write to
  /// @param parameters Additional control information that needs to be supplemented
  template <std::size_t N>
  Result write(const uint8_t id, const uint8_t memory_address, const std::array<uint8_t, N>& parameters) {
    return write_buffer(id, memory_address, parameters, kInstructionWrite).and_then([&] { return read_response(id); });
  }

 private:
  std::unique_ptr<SerialPort> serial_port_;

  Result read_response(uint8_t id);
  Result check_head();

  template <std::size_t N>
  Result write_buffer(const uint8_t id,
                      const uint8_t memory_address,
                      const std::array<uint8_t, N>& parameters,
                      const uint8_t instruction) {
    std::array<uint8_t, 7 + N> write_buf{};
    write_buf[2] = id;
    write_buf[3] = 3 + N;  // Message length
    write_buf[4] = instruction;
    write_buf[5] = memory_address;
    for (size_t i = 0; i < N; ++i) {
      write_buf[6 + i] = parameters[i];
    }
    write_buf[6 + N] = ~sum_bytes(write_buf);
    // Set these two after calculating the checksum
    write_buf[0] = 0xFF;
    write_buf[1] = 0xFF;

    return serial_port_->write(write_buf);
  }
};
}  // namespace feetech_hardware_interface
