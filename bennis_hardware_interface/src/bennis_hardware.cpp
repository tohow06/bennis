// Copyright (c) 2025, 
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include "bennis_hardware_interface/bennis_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bennis_hardware_interface
{

hardware_interface::CallbackReturn BennisHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.left_back_wheel_name = info_.hardware_parameters["left_back_wheel_name"];
  cfg_.right_back_wheel_name = info_.hardware_parameters["right_back_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  RCLCPP_INFO(logger_, "Using device: %s at baud rate: %d", cfg_.device.c_str(), cfg_.baud_rate);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  lb_wheel_.setup(cfg_.left_back_wheel_name, cfg_.enc_counts_per_rev);
  rb_wheel_.setup(cfg_.right_back_wheel_name, cfg_.enc_counts_per_rev);
  
  arduino_.open(cfg_.device, cfg_.baud_rate);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BennisHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
    // if (arduino_.ping()) {
    //   RCLCPP_INFO(logger_, "Arduino is ready.");
    // } else {
    //   RCLCPP_WARN(logger_, "Arduino is not responding.");
    //   return CallbackReturn::FAILURE;
    // }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BennisHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(lb_wheel_.name, hardware_interface::HW_IF_POSITION, &lb_wheel_.pos));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(rb_wheel_.name, hardware_interface::HW_IF_POSITION, &rb_wheel_.pos));


  state_interfaces.emplace_back(
    hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(lb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lb_wheel_.vel));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(rb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rb_wheel_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BennisHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(lb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lb_wheel_.cmd));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(rb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rb_wheel_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn BennisHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands
  RCLCPP_INFO(logger_, "Activating hardware interface...");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BennisHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  arduino_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BennisHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states

  
  auto speed = arduino_.read_speeds();

  if (speed) {

    // RCLCPP_INFO(logger_, "Read speeds: left=%d, right=%d, left_back=%d, right_back=%d",
    //   speed->fl, speed->fr, speed->rl, speed->rr);

    l_wheel_.vel = static_cast<double>(speed->fl);
    r_wheel_.vel = static_cast<double>(speed->fr);
    lb_wheel_.vel = static_cast<double>(speed->rl);
    rb_wheel_.vel = static_cast<double>(speed->rr);
    return hardware_interface::return_type::OK;
  }
  else {
    RCLCPP_WARN(logger_, "Failed to read speeds from Arduino.");
    return hardware_interface::return_type::ERROR;  // Use ERROR to indicate failure to read
  }
  
}

hardware_interface::return_type BennisHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'
  const Speeds speeds = {
    static_cast<int16_t>(l_wheel_.cmd), 
    static_cast<int16_t>(r_wheel_.cmd),
    static_cast<int16_t>(lb_wheel_.cmd), 
    static_cast<int16_t>(rb_wheel_.cmd)
  };
  arduino_.send_motor_speeds(speeds);
  return hardware_interface::return_type::OK;
}

}  // namespace bennis_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bennis_hardware_interface::BennisHardware, hardware_interface::SystemInterface)
