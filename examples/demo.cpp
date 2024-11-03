#include <spdlog/spdlog.h>

#include <feetech_hardware_interface/communication_protocol.hpp>
#include <iostream>
#include <range/v3/all.hpp>
#include <thread>
#include <tl_expected/expected.hpp>
#include <unordered_map>

using namespace std::chrono_literals;
using namespace feetech_hardware_interface;

std::string get_input(const std::string_view prompt) {
  std::string input;
  spdlog::info("{}", prompt);
  std::getline(std::cin, input);
  return input;
}

static constexpr auto kSleepTime = 100ms;

void print_models(CommunicationProtocol& communication_protocol) {
  for (std::size_t id = 0; id <= kMaxServoId; id++) {
    communication_protocol.read_model_number(id)
        .and_then(get_model_name)
        .and_then([=](const auto model_name) {
          spdlog::info("Servo ID: {} - Model name: {}", id, model_name);
          return Result{};
        })
        .or_else([=](const std::string& error) { spdlog::debug("Servo ID: {} - Error: {}", id, error); });
  }
}

void ping(CommunicationProtocol& communication_protocol) {
  const auto id = std::stoi(get_input("Enter servo ID: "));
  communication_protocol.ping(id)
      .and_then([] {
        spdlog::info("Ping success");
        return Result{};
      })
      .or_else([](const std::string& error) { spdlog::error("Ping failed: [{}]", error); });
}

void sync_read_position(CommunicationProtocol& communication_protocol) {
  const auto num_servos = std::stoul(get_input("Enter number of servos: "));
  const auto ids = ranges::views::iota(1ul, num_servos + 1) | ranges::to<std::vector<uint8_t>>();
  std::vector<std::array<uint8_t, 2>> positions(num_servos, {0, 0});

  while (true) {
    communication_protocol.sync_read(ids, SMS_STS_PRESENT_POSITION_L, &positions)
        .or_else([&](const std::string& error) {
          throw std::runtime_error(fmt::format("Failed to read position [ids={}]", ids, error));
        });
    spdlog::info("Position: {}", positions | ranges::views::transform([](const auto& position) {
                                   return from_sts(WordBytes{.low = position[0], .high = position[1]});
                                 }) | ranges::views::transform(to_angle));
    std::this_thread::sleep_for(kSleepTime);
  }
}

void read_position(CommunicationProtocol& communication_protocol) {
  const auto id = std::stoi(get_input("Enter servo ID: "));
  while (true) {
    const auto position = to_angle(communication_protocol.read_position(id)
                                       .or_else([](const std::string& error) { throw std::runtime_error(error); })
                                       .value());
    spdlog::info("Position: {:.3f}°", position);
    std::this_thread::sleep_for(kSleepTime);
  }
}

void sync_write_position(CommunicationProtocol& communication_protocol) {
  const auto num_servos = std::stoul(get_input("Enter number of servos: "));
  const auto ids = ranges::views::iota(1ul, num_servos + 1) | ranges::to<std::vector<uint8_t>>();
  std::vector<int> speeds(num_servos, 0);
  std::vector<int> accelerations(num_servos, 0);

  while (true) {
    const auto desired_joint_position = std::stoi(get_input("Enter desired joint position: "));
    std::vector<int> positions(num_servos, from_angle(desired_joint_position));

    spdlog::info("Setting positions to {}", positions);
    communication_protocol.sync_write_position(ids, positions, speeds, accelerations)
        .or_else([=](const std::string& error) {
          throw std::runtime_error(fmt::format("Failed to set position [ids={}]", ids, error));
        });
  }
}

void reg_write_position(CommunicationProtocol& communication_protocol) {
  const auto num_servos = std::stoi(get_input("Enter number of servos: "));
  while (true) {
    const auto desired_joint_position = std::stoi(get_input("Enter desired joint position: "));
    const auto data = from_angle(desired_joint_position);

    spdlog::info("Setting position to {}°: {}", desired_joint_position, data);
    for (uint8_t servo_id = 1; servo_id <= num_servos; servo_id++) {
      communication_protocol.reg_write_position(servo_id, data, 0, 0).or_else([=](const std::string& error) {
        throw std::runtime_error(fmt::format("Failed to set position [id={}][{}]", servo_id, error));
      });
    }
    get_input("Press enter to continue");
    communication_protocol.reg_write_action().or_else([](const std::string& error) {
      throw std::runtime_error(fmt::format("Failed to set position action [{}]", error));
    });
  }
}

void write_position(CommunicationProtocol& communication_protocol) {
  const auto id = std::stoi(get_input("Enter servo ID: "));
  communication_protocol.set_mode(id, OperationMode::kPosition).or_else([](const std::string& error) {
    throw std::runtime_error(error);
  });
  while (true) {
    const auto desired_joint_position = std::stoi(get_input("Enter desired joint position: "));
    const auto data = from_angle(desired_joint_position);
    spdlog::info("Setting position to {}°: {}", desired_joint_position, data);

    if (!communication_protocol.write_position(id, data, 0, 0)) {
      spdlog::error("Failed to set position");
    }

    double position = -1.;
    while (std::abs(position - desired_joint_position) > 1e2) {
      position =
          to_angle(communication_protocol.read_position(id)
                       .or_else([](const std::string& error) -> Expected<int> { throw std::runtime_error(error); })
                       .value());
      spdlog::info("Current position: {:.3f}°", position);
      std::this_thread::sleep_for(kSleepTime);
    }
  }
}

void read_speed(CommunicationProtocol& communication_protocol) {
  const auto id = std::stoi(get_input("Enter servo ID: "));
  while (true) {
    const auto speed = to_angle(communication_protocol.read_speed(id)
                                    .or_else([](const std::string& error) { throw std::runtime_error(error); })
                                    .value());
    spdlog::info("Speed: {:.3f}", speed);
    std::this_thread::sleep_for(kSleepTime);
  }
}

const std::unordered_map<std::string_view, void (*)(CommunicationProtocol&)> kExamples = {
    // clang-format off
    {"ping", ping},
    {"read_position", read_position},
    {"write_position", write_position},
    {"read_speed", read_speed},
    {"print_models", print_models},
    {"reg_write_position", reg_write_position},
    {"sync_write_position", sync_write_position},
    {"sync_read_position", sync_read_position},
    // clang-format on
};

int main(int argc, char** argv) {
  if (argc != 3) {
    spdlog::error("Usage: {} <example_name> <port_name>", argv[0]);
    return EXIT_FAILURE;
  }

  const auto example = kExamples.find(argv[1]);
  if (example == kExamples.end()) {
    std::vector<std::string_view> keys;
    keys.reserve(kExamples.size());
    for (const auto& [key, _] : kExamples) {
      keys.push_back(key);
    }
    spdlog::error("Invalid example name: {} - Available examples: {}", argv[1], keys);
    return EXIT_FAILURE;
  }

  const std::string port_name = argv[2];

  auto serial_port = std::make_unique<SerialPort>(port_name);
  serial_port->configure().and_then([&] { return serial_port->open(); }).or_else([](const std::string& error) {
    throw std::runtime_error(error);
  });

  auto communication_protocol = CommunicationProtocol(std::move(serial_port));
  example->second(communication_protocol);
}
