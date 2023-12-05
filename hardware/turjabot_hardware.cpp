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

#include "hw_turjabot/turjabot_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hw_turjabot
{
hardware_interface::CallbackReturn TurjabotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  
  RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Starting init func ...please wait...");
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_drive_wheel_name = info_.hardware_parameters["left_drive_wheel_name"];
  cfg_.right_drive_wheel_name = info_.hardware_parameters["right_drive_wheel_name"];
  cfg_.left_steer_wheel_name = info_.hardware_parameters["left_steer_wheel_name"];
  cfg_.right_steer_wheel_name = info_.hardware_parameters["right_steer_wheel_name"];
  cfg_.camera_pan_name = info_.hardware_parameters["camera_pan_name"];
  cfg_.camera_tilt_name = info_.hardware_parameters["camera_tilt_name"];
  //cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  wheel_dl_.setup(cfg_.left_drive_wheel_name, cfg_.enc_counts_per_rev);
  wheel_dr_.setup(cfg_.right_drive_wheel_name, cfg_.enc_counts_per_rev);
  
  servo_l_steer_.setup(cfg_.left_steer_wheel_name);
  servo_r_steer_.setup(cfg_.right_steer_wheel_name);
  servo_cam_pan_.setup(cfg_.camera_pan_name);
  servo_cam_tilt_.setup(cfg_.camera_tilt_name);


    // TurjabotHardware has 2 states, 1 command for drive wheels.
    //                      Pos, Vel states, Vel command
    //                  1 state, 1 command for steer and cam servos.
    //                      Pos states, Pos command

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // All joints should have only one Command Interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("TurjabotHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Drive wheels have Velocity Command Interface.
    if(joint.name.c_str() == cfg_.left_drive_wheel_name || joint.name.c_str() == cfg_.right_drive_wheel_name) {
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TurjabotHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      //  Others (Servos) have Position Command Interface.
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TurjabotHardware"),
          "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    
    // Drive wheels have 2 State Interfaces, for Position and Velocity
    if(joint.name.c_str() == cfg_.left_drive_wheel_name || joint.name.c_str() == cfg_.right_drive_wheel_name) {
      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TurjabotHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TurjabotHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TurjabotHardware"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      // Others (Servos) have 1 State Interface, for Position.
      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TurjabotHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("TurjabotHardware"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }


  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TurjabotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_dl_.name, hardware_interface::HW_IF_POSITION, &wheel_dl_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_dl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_dl_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_dr_.name, hardware_interface::HW_IF_POSITION, &wheel_dr_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_dr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_dr_.vel));

  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    servo_l_steer_.name, hardware_interface::HW_IF_POSITION, &servo_l_steer_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    servo_r_steer_.name, hardware_interface::HW_IF_POSITION, &servo_r_steer_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    servo_cam_pan_.name, hardware_interface::HW_IF_POSITION, &servo_cam_pan_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    servo_cam_tilt_.name, hardware_interface::HW_IF_POSITION, &servo_cam_tilt_.pos));

  for (const hardware_interface::StateInterface & ifc : state_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Name of interface: %s", ifc.get_name().c_str());
  }
  
  
  RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "\r\nState Interfaces exported!\r\n");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TurjabotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_dl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_dl_.vel));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_dr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_dr_.vel));


  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    servo_l_steer_.name, hardware_interface::HW_IF_POSITION, &servo_l_steer_.pos));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    servo_r_steer_.name, hardware_interface::HW_IF_POSITION, &servo_r_steer_.pos));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    servo_cam_pan_.name, hardware_interface::HW_IF_POSITION, &servo_cam_pan_.pos));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    servo_cam_tilt_.name, hardware_interface::HW_IF_POSITION, &servo_cam_tilt_.pos));

  
  for (const hardware_interface::CommandInterface & ifc : command_interfaces)
  {
    RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Name of interface: %s", ifc.get_name().c_str());
  }
  
  RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "\r\nCommand Interfaces exported!\r\n");

  return command_interfaces;
}

hardware_interface::CallbackReturn TurjabotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Activating ...please wait...");



  // Initialize comms_ here, so connect and start the robot hardware




  RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TurjabotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Deactivating ...please wait...");



  // Stop comms_ here, so disconnect and stop the robot hardware



  RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TurjabotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  comms_.read_encoder_values(wheel_dl_.enc, wheel_dr_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_dl_.pos;
  wheel_dl_.pos = wheel_dl_.calc_enc_angle();
  wheel_dl_.vel = (wheel_dl_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_dr_.pos;
  wheel_dr_.pos = wheel_dr_.calc_enc_angle();
  wheel_dr_.vel = (wheel_dr_.pos - pos_prev) / delta_seconds;


  // TODO: Servo read position


  

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hw_turjabot ::TurjabotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int motor_dl_counts_per_loop = wheel_dl_.cmd / wheel_dl_.rads_per_count; // / cfg_.loop_rate;
  int motor_dr_counts_per_loop = wheel_dr_.cmd / wheel_dr_.rads_per_count; // / cfg_.loop_rate;
  comms_.set_motor_values(motor_dl_counts_per_loop, motor_dr_counts_per_loop);

  
  // TODO: Servo write position
  if(wheel_dl_.cmd != 0) {
    RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Got write request, cmd: %g", wheel_dl_.cmd);
  }
  if(wheel_dr_.cmd != 0) {
    RCLCPP_INFO(rclcpp::get_logger("TurjabotHardware"), "Got write request, cmd: %g", wheel_dr_.cmd);
  }
  

  return hardware_interface::return_type::OK;
}

}  // namespace hw_turjabot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hw_turjabot::TurjabotHardware, hardware_interface::SystemInterface)
