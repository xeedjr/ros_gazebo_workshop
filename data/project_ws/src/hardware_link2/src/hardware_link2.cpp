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

#include <limits>
#include <vector>

#include "hardware_link2/hardware_link2.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <fcntl.h>   // File control definitions
#include <unistd.h>  // UNIX standard function definitions
#include <termios.h> // POSIX terminal control definitions
#include <errno.h>   // Error number definitions
#include <string.h>  // String function definitions

namespace hardware_link2
{
hardware_interface::CallbackReturn HardwareLinkInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  cfg.device = info_.hardware_parameters["device"];
  cfg.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  // TODO(anyone): read parameters and initialize the hardware
//  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Init ...please wait...");

    // Open the serial port
    fd = open(cfg.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error " << errno << " opening " << cfg.device << ": " << strerror(errno) << std::endl;
        return CallbackReturn::ERROR;
    }

    // Set the baud rate
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return CallbackReturn::ERROR;
    }

    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    // Setting raw mode
    cfmakeraw(&tty); // This sets the terminal to raw mode

    // Additional configuration
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo,
                                                // no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // read doesn't block
    tty.c_cc[VTIME] = 30;                       // 3 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls,
                                                // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        return CallbackReturn::ERROR;
    }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HardwareLinkInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Configure ...please wait...");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HardwareLinkInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].name == "left_wheel_joint") {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          // TODO(anyone): insert correct interfaces
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &wheels[0].state_pos));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels[0].state_vel));

          RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "export state %s", info_.joints[i].name.c_str());
    };
    if (info_.joints[i].name == "right_wheel_joint") {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          // TODO(anyone): insert correct interfaces
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &wheels[1].state_pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels[1].state_vel));

          RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "export state %s", info_.joints[i].name.c_str());
    };    
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareLinkInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].name == "left_wheel_joint") {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        // TODO(anyone): insert correct interfaces
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &(wheels[0].cmd_vel)));
    };
    if (info_.joints[i].name == "right_wheel_joint") {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        // TODO(anyone): insert correct interfaces
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &wheels[1].cmd_vel));
    };

    RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "export command %s", info_.joints[i].name.c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HardwareLinkInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HardwareLinkInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

  close(fd);

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HardwareLinkInterface::read(
  const rclcpp::Time & time /*time*/, const rclcpp::Duration & period/*period*/)
{
  // TODO(anyone): read robot states

  wheels[0].state_pos_prev = wheels[0].state_pos;
  wheels[1].state_pos_prev = wheels[1].state_pos;
std::cerr << "read state enter: " << std::endl;

  // Clear the buffer
  tcflush(fd , TCIFLUSH);

  // Send "pos\n\r"
  const char *msg = "pos\r\n";
  if (::write(fd, msg, strlen(msg)) < 0) {
      std::cerr << "Error " << errno << " writing to " << cfg.device << ": " << strerror(errno) << std::endl;
      close(fd);
      hardware_interface::return_type::ERROR;
  }


  // Read until '\r'   Echo
  char bufp[256];
  auto sizep = read_buffer(bufp, sizeof(bufp));
//    std::cerr << "read :" << bufp << ":end size: "<< sizep << std::endl;

// Read until '\r'   Pos
  char buf[256];
  auto size = read_buffer(buf, sizeof(buf));
//    std::cerr << "read :" << buf << ":end size: " << size << std::endl;

// Read until '\r'   Ok
  char buft[256];
  auto sizet = read_buffer(buft, sizeof(buft));
//    std::cerr << "read :" << buft << ":end size: " << sizet << std::endl;


  double w1_pos = 0;
  double w2_pos = 0;

  double w1_vel = 0;
  double w2_vel = 0;

  double w1_pwm = 0;
  double w2_pwm = 0;

//  char* d = "0.000000 0.000000";

  auto res = sscanf(buf, "%lf %lf %lf %lf %lf %lf", &w1_pos, &w2_pos, &w1_vel, &w2_vel, &w1_pwm, &w2_pwm);

  if (res != 6) {
    std::cerr << "sscanf :" << res << "str: " << buf << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  wheels[0].state_pos = w1_pos;
  wheels[1].state_pos = w2_pos*-1.0;

//  double wposdelta0 =  wheels[0].state_pos - wheels[0].state_pos_prev;
//  double wposdelta1 =  wheels[1].state_pos - wheels[1].state_pos_prev;

//  double koef = 1.0 / period.seconds();
  //std::cerr << "koef :" << koef << std::endl;
  //std::cerr << "period :" << period.seconds() << std::endl;
  //std::cerr << "wposdelta0 :" << wposdelta0 << std::endl;

//  wheels[0].state_vel = wposdelta0 * koef;
//  wheels[1].state_vel = wposdelta1 * koef;
  wheels[0].state_vel = w1_vel;
  wheels[1].state_vel = w2_vel*-1.0;

//std::cerr << "res :  " << res << std::endl;
//std::cerr << "state :  " << w1 << " wwww " << w2 << ": " << std::endl;

  // Clear the buffer
  tcflush(fd , TCIFLUSH);

std::cerr << "read state exit:" << std::endl;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HardwareLinkInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::cerr << "write state enter:" << std::endl;

  // TODO(anyone): write robot's commands'
  // Clear the buffer
  tcflush(fd , TCIFLUSH);

  char buf[256];

  sprintf(buf, "set %lf %lf\r\n0", wheels[0].cmd_vel, wheels[1].cmd_vel*-1.0);
  if (::write(fd, buf, strlen(buf)) < 0) {
      std::cerr << "Error " << errno << " writing to " << cfg.device << ": " << strerror(errno) << std::endl;
      close(fd);
      hardware_interface::return_type::ERROR;
  }

  char bufp[256];
  auto sizep = read_buffer(bufp, sizeof(bufp));
//    std::cerr << "read :" << bufp << ":end"<< std::endl;

  char bufok[256];
  auto sizeok = read_buffer(bufok, sizeof(bufok));
//    std::cerr << "read :" << bufok << ":end"<< std::endl;

  // Clear the buffer
  tcflush(fd , TCIFLUSH);

std::cerr << "write state exit:" << std::endl;

  return hardware_interface::return_type::OK;
}

}  // namespace hardware_link2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hardware_link2::HardwareLinkInterface, hardware_interface::SystemInterface)
