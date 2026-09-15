// Copyright (c) 2026 OLO Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include <robotiq_driver/driver_exception.hpp>
#include <robotiq_driver/urcap/urcap_driver.hpp>

#include <rclcpp/logging.hpp>

namespace robotiq_driver
{
const auto kLogger = rclcpp::get_logger("UrcapDriver");

// URCap socket protocol variable names.
constexpr auto kActivateVariable = "ACT";
constexpr auto kAutoReleaseVariable = "ATR";
constexpr auto kGoToVariable = "GTO";
constexpr auto kPositionVariable = "POS";
constexpr auto kSpeedVariable = "SPE";
constexpr auto kForceVariable = "FOR";
constexpr auto kStatusVariable = "STA";
constexpr auto kObjectDetectionVariable = "OBJ";
constexpr auto kPositionRequestVariable = "PRE";

constexpr auto kAckResponse = "ack";

// URCap status register values.
constexpr uint8_t kGripperActivatedStatus = 3;
constexpr uint8_t kGripperResetStatus = 0;
constexpr uint8_t kObjectDetectionMoving = 0;
constexpr uint8_t kObjectDetectionAtDestination = 3;

// PickNik hardware_interface operational range used for radians conversion.
constexpr uint8_t kNominalMinPosition = 3;
constexpr uint8_t kNominalMaxPosition = 230;
constexpr uint8_t kNominalRange = kNominalMaxPosition - kNominalMinPosition;

// Slow, low-force moves used while measuring the open/closed endpoints.
constexpr uint8_t kCalibrationSpeed = 64;
constexpr uint8_t kCalibrationForce = 1;

// Socket I/O timeout applied to each send/recv call.
constexpr timeval kResponseTimeout = { 2, 0 };  // 2s, matches FZI default

// Activation and motion polling parameters.
constexpr auto kActivationTimeout = std::chrono::seconds{ 10 };
constexpr auto kMotionTimeout = std::chrono::seconds{ 30 };
constexpr auto kActivationPollPeriod = std::chrono::milliseconds{ 100 };
constexpr auto kMotionPollPeriod = std::chrono::milliseconds{ 10 };
constexpr auto kResetSettleTime = std::chrono::milliseconds{ 500 };
constexpr auto kActivationSettleTime = std::chrono::seconds{ 1 };

UrcapDriver::UrcapDriver(std::string robot_ip, uint16_t robot_port)
  : robot_ip_{ std::move(robot_ip) }, robot_port_{ robot_port }
{
}

UrcapDriver::~UrcapDriver()
{
  disconnect();
}

std::string UrcapDriver::send_command(const std::string& command)
{
  if (socket_fd_ < 0)
  {
    throw DriverException{ "Not connected to the URCap socket server" };
  }

  const std::string request = command + "\n";

  // Send the full request, handling partial writes.
  size_t bytes_sent = 0;
  while (bytes_sent < request.size())
  {
    const ssize_t result = send(socket_fd_, request.data() + bytes_sent, request.size() - bytes_sent, MSG_NOSIGNAL);
    if (result <= 0)
    {
      throw DriverException{ "Failed to send command: " + command + ": " + std::strerror(errno) };
    }
    bytes_sent += static_cast<size_t>(result);
  }

  // URCap replies with a single short message and no trailing newline (matches FZI adapter).
  char buffer[1024];
  const ssize_t bytes_read = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
  if (bytes_read <= 0)
  {
    throw DriverException{ "Failed to read response to command: " + command + ": " + std::strerror(errno) };
  }
  buffer[bytes_read] = '\0';

  std::string response{ buffer };
  while (!response.empty() && (response.back() == '\n' || response.back() == '\r'))
  {
    response.pop_back();
  }
  return response;
}

bool UrcapDriver::connect()
{
  if (socket_fd_ >= 0)
  {
    return true;
  }

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* addresses = nullptr;
  const auto port = std::to_string(robot_port_);
  if (getaddrinfo(robot_ip_.c_str(), port.c_str(), &hints, &addresses) != 0)
  {
    RCLCPP_ERROR(kLogger, "Cannot resolve gripper address %s:%s", robot_ip_.c_str(), port.c_str());
    return false;
  }

  for (addrinfo* address = addresses; address != nullptr; address = address->ai_next)
  {
    const int fd = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (fd < 0)
    {
      continue;
    }
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &kResponseTimeout, sizeof(kResponseTimeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &kResponseTimeout, sizeof(kResponseTimeout));
    if (::connect(fd, address->ai_addr, address->ai_addrlen) == 0)
    {
      socket_fd_ = fd;
      break;
    }
    close(fd);
  }
  freeaddrinfo(addresses);

  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(kLogger, "Cannot connect to the URCap socket server at %s:%s", robot_ip_.c_str(), port.c_str());
    return false;
  }

  RCLCPP_INFO(kLogger, "Connected to the URCap socket server at %s:%s", robot_ip_.c_str(), port.c_str());
  return true;
}

void UrcapDriver::disconnect()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void UrcapDriver::set_slave_address(uint8_t /*slave_address*/)
{
  // The slave address is a Modbus RTU concept with no URCap equivalent.
}

void UrcapDriver::activate()
{
  RCLCPP_INFO(kLogger, "Activate...");

  // If activation is already complete (STA == 3), do not reactivate
  const bool already_activated = get_variable(kStatusVariable) == kGripperActivatedStatus;
  if (!already_activated)
  {
    reset();

    // Set ACT to 1 to begin gripper activation.
    set_variable(kActivateVariable, 1);
    std::this_thread::sleep_for(kActivationSettleTime);

    const auto deadline = std::chrono::steady_clock::now() + kActivationTimeout;
    while (get_variable(kActivateVariable) != 1 || get_variable(kStatusVariable) != kGripperActivatedStatus)
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        throw DriverException{ "Timeout while waiting for the gripper to activate" };
      }
      std::this_thread::sleep_for(kActivationPollPeriod);
    }
  }

  auto_calibrate();

  // Calibration ends open. Store that pose with the operational speed/force so
  // the comms thread does not immediately re-issue GTO=1 against the open stop.
  last_command_ = GripperCommand{ min_position_, commanded_gripper_speed_, commanded_gripper_force_ };
}

void UrcapDriver::deactivate()
{
  RCLCPP_INFO(kLogger, "Deactivate...");

  reset();
}

void UrcapDriver::set_gripper_position(uint8_t pos)
{
  const GripperCommand command{ to_raw_position(pos), commanded_gripper_speed_, commanded_gripper_force_ };
  if (last_command_ == command)
  {
    return;
  }

  set_variables(std::string{ kPositionVariable } + " " + std::to_string(command.position) + " " + kSpeedVariable + " " +
                std::to_string(command.speed) + " " + kForceVariable + " " + std::to_string(command.force) + " " +
                kGoToVariable + " 1");
  last_command_ = command;
}

uint8_t UrcapDriver::get_gripper_position()
{
  return to_nominal_position(get_variable(kPositionVariable));
}

bool UrcapDriver::gripper_is_moving()
{
  // Object detection status: 0 indicates fingers in motion.
  return get_variable(kObjectDetectionVariable) == kObjectDetectionMoving;
}

void UrcapDriver::set_speed(uint8_t speed)
{
  commanded_gripper_speed_ = speed;
}

void UrcapDriver::set_force(uint8_t force)
{
  commanded_gripper_force_ = force;
}

void UrcapDriver::reset()
{
  const auto deadline = std::chrono::steady_clock::now() + kActivationTimeout;
  do
  {
    set_variable(kActivateVariable, 0);
    set_variable(kAutoReleaseVariable, 0);
    if (get_variable(kActivateVariable) == 0 && get_variable(kStatusVariable) == kGripperResetStatus)
    {
      std::this_thread::sleep_for(kResetSettleTime);
      return;
    }
    std::this_thread::sleep_for(kActivationPollPeriod);
  } while (std::chrono::steady_clock::now() < deadline);

  throw DriverException{ "Timeout while waiting for the gripper to reset" };
}

void UrcapDriver::auto_calibrate()
{
  RCLCPP_INFO(kLogger, "Calibrating gripper endpoints...");

  // Close as far as possible and record the closed endpoint.
  const auto closed = move_and_wait_for_position(max_position_, kCalibrationSpeed, kCalibrationForce);
  if (closed.second != kObjectDetectionAtDestination)
  {
    throw DriverException{ "Calibration failed while closing because an object was detected" };
  }
  max_position_ = closed.first;

  // Open as far as possible and record the open endpoint.
  const auto opened = move_and_wait_for_position(min_position_, kCalibrationSpeed, kCalibrationForce);
  if (opened.second != kObjectDetectionAtDestination)
  {
    throw DriverException{ "Calibration failed while opening because an object was detected" };
  }
  min_position_ = opened.first;

  if (max_position_ <= min_position_)
  {
    throw DriverException{ "Calibration measured an invalid position range" };
  }

  RCLCPP_INFO(kLogger, "Gripper auto-calibrated to [%u, %u]", min_position_, max_position_);
}

std::pair<uint8_t, uint8_t> UrcapDriver::move_and_wait_for_position(uint8_t position, uint8_t speed, uint8_t force)
{
  const auto initial_position = get_variable(kPositionVariable);
  const bool motion_expected = initial_position != position;
  set_variables(std::string{ kPositionVariable } + " " + std::to_string(position) + " " + kSpeedVariable + " " +
                std::to_string(speed) + " " + kForceVariable + " " + std::to_string(force) + " " + kGoToVariable +
                " 1");

  // Wait until the gripper acknowledges the requested position.
  auto deadline = std::chrono::steady_clock::now() + kMotionTimeout;
  while (get_variable(kPositionRequestVariable) != position)
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      throw DriverException{ "Timeout while waiting for the gripper to acknowledge the position request" };
    }
    std::this_thread::sleep_for(kMotionPollPeriod);
  }

  // Wait for motion to start before accepting a previous completed status.
  if (motion_expected)
  {
    deadline = std::chrono::steady_clock::now() + kMotionTimeout;
    while (get_variable(kObjectDetectionVariable) != kObjectDetectionMoving &&
           get_variable(kPositionVariable) == initial_position)
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        throw DriverException{ "Timeout while waiting for the gripper motion to start" };
      }
      std::this_thread::sleep_for(kMotionPollPeriod);
    }
  }

  // Wait until the fingers stop moving.
  deadline = std::chrono::steady_clock::now() + kMotionTimeout;
  uint8_t object_status = get_variable(kObjectDetectionVariable);
  while (object_status == kObjectDetectionMoving)
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      throw DriverException{ "Timeout while waiting for the gripper motion to complete" };
    }
    std::this_thread::sleep_for(kMotionPollPeriod);
    object_status = get_variable(kObjectDetectionVariable);
  }

  return { get_variable(kPositionVariable), object_status };
}

uint8_t UrcapDriver::to_raw_position(uint8_t pos) const
{
  const int clamped = std::clamp(static_cast<int>(pos), static_cast<int>(kNominalMinPosition),
                                 static_cast<int>(kNominalMaxPosition));
  if (max_position_ == min_position_)
  {
    return min_position_;
  }

  return static_cast<uint8_t>(min_position_ +
                              (clamped - kNominalMinPosition) * (max_position_ - min_position_) / kNominalRange);
}

uint8_t UrcapDriver::to_nominal_position(uint8_t pos) const
{
  const int clamped =
      std::clamp(static_cast<int>(pos), static_cast<int>(min_position_), static_cast<int>(max_position_));
  if (max_position_ == min_position_)
  {
    return kNominalMinPosition;
  }

  return static_cast<uint8_t>(kNominalMinPosition +
                              (clamped - min_position_) * kNominalRange / (max_position_ - min_position_));
}

void UrcapDriver::set_variables(const std::string& variables)
{
  const auto response = send_command("SET " + variables);
  if (response != kAckResponse)
  {
    throw DriverException{ "Gripper did not acknowledge SET " + variables + ", response: " + response };
  }
}

void UrcapDriver::set_variable(const std::string& variable, uint8_t value)
{
  set_variables(variable + " " + std::to_string(value));
}

uint8_t UrcapDriver::get_variable(const std::string& variable)
{
  const auto response = send_command("GET " + variable);

  // Process the response.
  const auto separator = response.find(' ');
  if (separator == std::string::npos || response.substr(0, separator) != variable)
  {
    throw DriverException{ "Unexpected response to GET " + variable + ": " + response };
  }
  return static_cast<uint8_t>(std::stoul(response.substr(separator + 1)));
}
}  // namespace robotiq_driver
