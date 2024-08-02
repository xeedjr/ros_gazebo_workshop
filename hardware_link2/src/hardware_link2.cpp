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

namespace hardware_link2
{
hardware_interface::CallbackReturn HardwareLinkInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }


  // TODO(anyone): read parameters and initialize the hardware
//  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Init ...please wait...");

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

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type HardwareLinkInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HardwareLinkInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'

  return hardware_interface::return_type::OK;
}

}  // namespace hardware_link2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hardware_link2::HardwareLinkInterface, hardware_interface::SystemInterface)
