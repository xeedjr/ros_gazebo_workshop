// Copyright (c) 2024, Bogdan
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HARDWARE_LINK2__HARDWARE_LINK2_HPP_
#define HARDWARE_LINK2__HARDWARE_LINK2_HPP_

#include <string>
#include <vector>

#include <iostream>
#include <fcntl.h>   // File control definitions
#include <unistd.h>  // UNIX standard function definitions
#include <termios.h> // POSIX terminal control definitions
#include <errno.h>   // Error number definitions
#include <string.h>  // String function definitions

#include "hardware_link2/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_link2
{

class Cfg {
public:
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
};

class Wheel {

public:
    double state_vel = 0;
    double state_pos = 0;
    double state_pos_prev = 0;
    double cmd_vel = 0;

    Wheel() {
    };
};

class IMU {

public:
  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_ = {0};

    IMU() {
    };
};

class HardwareLinkInterface : public hardware_interface::SystemInterface
{

  int read_buffer(char* buff, int size ) {
    int index = 0;
    while (true) {
//      std::cerr << "read state 1:" << std::endl;
        int n = ::read(fd, &buff[index], 1);
//        std::cerr << "read state 2: " << n << "  " << (int) buff[index]<< std::endl;
        if (n < 0) {
            std::cerr << "Error " << errno << " reading from " << cfg.device << ": " << strerror(errno) << std::endl;
            return -1;
        }
        if (n == 0) {
            // No data available (timeout or end-of-file)
            std::cerr << "Timeout" << std::endl;
            return 0;
        }
        if (buff[index] == '\n') {
            // End of line
            index++;
            break;
        }
        index++;
        if (index >= size - 1) {
            // Buffer overflow protection
            std::cerr << "Buffer overflow" << std::endl;
            return -1;
        }
    }
    buff[index] = '\0'; // Null-terminate the string

    return index;
  };

public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  Wheel wheels[2];
  IMU imu_;
  Cfg cfg;

  int fd;
//  std::vector<double> hw_commands_;
//  std::vector<double> hw_states_;
};

}  // namespace hardware_link2

#endif  // HARDWARE_LINK2__HARDWARE_LINK2_HPP_
