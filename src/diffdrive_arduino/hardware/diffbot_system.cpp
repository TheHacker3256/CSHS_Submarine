// Copyright 2021 ros2_control Development Team
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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  cfg_.front_left_drone = info_.hardware_parameters["front_left_drone"];
  cfg_.front_right_drone = info_.hardware_parameters["front_right_drone"];
  cfg_.rear_left_drone = info_.hardware_parameters["rear_left_drone"];
  cfg_.rear_right_drone = info_.hardware_parameters["rear_right_drone"];
  cfg_.forward_left_motor = info_.hardware_parameters["forward_left_motor"];
  cfg_.forward_right_motor = info_.hardware_parameters["forward_right_motor"];
  
  fld.name = cfg_.front_left_drone;
  frd.name = cfg_.front_right_drone;
  rld.name = cfg_.rear_left_drone;
  rrd.name = cfg_.rear_right_drone;

  flm.name = cfg_.forward_left_motor;
  frm.name = cfg_.forward_right_motor;



  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* //state interface that is not in use
std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

//front left drone motor state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(fld.name, hardware_interface::HW_IF_POSITION, &fld.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fld.name, hardware_interface::HW_IF_VELOCITY, &fld.vel));
//front right drone motor state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(frd.name, hardware_interface::HW_IF_POSITION, &frd.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(frd.name, hardware_interface::HW_IF_VELOCITY, &frd.vel));
//rear left drone motor state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(rld.name, hardware_interface::HW_IF_POSITION, &rld.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(rld.name, hardware_interface::HW_IF_VELOCITY, &rld.vel));
//rear right drone motor state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(rrd.name, hardware_interface::HW_IF_POSITION, &rrd.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(rrd.name, hardware_interface::HW_IF_VELOCITY, &rrd.vel));
//forward left thrust motor state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(flm.name, hardware_interface::HW_IF_POSITION, &flm.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(flm.name, hardware_interface::HW_IF_VELOCITY, &flm.vel));
//front right thrust motor state interface
  state_interfaces.emplace_back(hardware_interface::StateInterface(frd.name, hardware_interface::HW_IF_POSITION, &frd.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(frd.name, hardware_interface::HW_IF_VELOCITY, &frd.vel));

  return state_interfaces;
} 
*/

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(fld.name, hardware_interface::HW_IF_VELOCITY, &fld.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(frd.name, hardware_interface::HW_IF_VELOCITY, &frd.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(rld.name, hardware_interface::HW_IF_VELOCITY, &rld.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(rrd.name, hardware_interface::HW_IF_VELOCITY, &rrd.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(flm.name, hardware_interface::HW_IF_VELOCITY, &flm.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(frm.name, hardware_interface::HW_IF_VELOCITY, &frm.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");

  initGPIO(); //on configure

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");

  //on cleanup

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");

  initMotors(); //on activate
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");

  deactivateGPIO(); //on deactivate

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  
  //on read of state interfaces

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  //on write of hardware interfaces
  
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
