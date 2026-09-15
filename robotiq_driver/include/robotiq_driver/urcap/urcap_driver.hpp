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

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include <robotiq_driver/driver.hpp>

/**
 * @brief This class is responsible for communicating with the gripper via a URCap socket server, and maintaining a
 * record of the gripper's current state.
 *
 */
namespace robotiq_driver
{
class UrcapDriver : public Driver
{
public:
  UrcapDriver(std::string robot_ip, uint16_t robot_port);
  ~UrcapDriver() override;

  bool connect() override;
  void disconnect() override;

  void set_slave_address(uint8_t slave_address) override;

  /** Activate the gripper with the specified operation mode and parameters. */
  void activate() override;

  /** Deactivate the gripper. */
  void deactivate() override;

  /**
   * @brief Commands the gripper to move to the desired position.
   * @param pos A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  void set_gripper_position(uint8_t pos) override;

  /**
   * @brief Return the current position of the gripper.
   * @throw DriverException on failure to successfully communicate with the URCap socket server
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  uint8_t get_gripper_position() override;

  /**
   * @brief Returns true if the gripper is currently moving, false otherwise.
   *
   */
  bool gripper_is_moving() override;

  /**
   * @brief Set the speed of the gripper.
   * @param speed A value between 0x00 (stopped) and 0xFF (full speed).
   */
  void set_speed(uint8_t speed) override;

  /**
   * @brief Set how forcefully the gripper opens or closes.
   * @param force A value between 0x00 (no force) or 0xFF (maximum force).
   */
  void set_force(uint8_t force) override;

private:
  /**
   * Send a command to the URCap socket server and wait for a single-line response.
   * If the response is not received within the timeout, a DriverException is thrown.
   * @param command The command string without trailing newline.
   * @return The response without the trailing newline, e.g. "POS 128".
   * @throw DriverException on socket failure or timeout.
   */
  std::string send_command(const std::string& command);

  /** Reset the gripper and wait for it to become inactive. */
  void reset();

  /**
   * @brief Measure the open and closed endpoints by slowly closing and opening the gripper.
   * @throw DriverException if calibration cannot reach either endpoint.
   */
  void auto_calibrate();

  /**
   * @brief Command a raw URCap position and wait until the motion completes.
   * @return The final raw position and object-detection status.
   * @throw DriverException on timeout or communication failure.
   */
  std::pair<uint8_t, uint8_t> move_and_wait_for_position(uint8_t position, uint8_t speed, uint8_t force);

  /**
   * @brief Map a PickNik nominal position onto the calibrated URCap range.
   * @param pos A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  uint8_t to_raw_position(uint8_t pos) const;

  /**
   * @brief Map a raw URCap position back onto the PickNik nominal range.
   * @return uint8_t A value between 0x00 (fully open) and 0xFF (fully closed).
   */
  uint8_t to_nominal_position(uint8_t pos) const;

  /**
   * @brief Set one or more variables and check the gripper acknowledged them.
   * @throw DriverException if the gripper does not acknowledge the command.
   */
  void set_variables(const std::string& variables);

  /**
   * @brief Send "SET <variable> <value>" and check the gripper acknowledged it.
   * @throw DriverException if the gripper does not acknowledge the command.
   */
  void set_variable(const std::string& variable, uint8_t value);

  /**
   * @brief Send "GET <variable>" and return the reported value.
   * @throw DriverException on failure to successfully communicate with the URCap socket server
   * @return uint8_t The value reported by the gripper.
   */
  uint8_t get_variable(const std::string& variable);

  std::string robot_ip_;
  uint16_t robot_port_;
  int socket_fd_ = -1;

  uint8_t commanded_gripper_speed_ = 0xFF;
  uint8_t commanded_gripper_force_ = 0xFF;

  struct GripperCommand
  {
    uint8_t position = 0;
    uint8_t speed = 0;
    uint8_t force = 0;

    bool operator==(const GripperCommand& other) const
    {
      return position == other.position && speed == other.speed && force == other.force;
    }
  };

  std::optional<GripperCommand> last_command_;

  // Measured open/closed endpoints after activation calibration.
  uint8_t min_position_ = 0;
  uint8_t max_position_ = 255;
};
}  // namespace robotiq_driver
