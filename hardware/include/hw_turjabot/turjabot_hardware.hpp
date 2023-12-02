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

#ifndef HW_TURJABOT__DIFFBOT_SYSTEM_HPP_
#define HW_TURJABOT__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hw_turjabot/visibility_control.h"

#include "hw_turjabot/turjabot_comms.hpp"
#include "hw_turjabot/wheel.hpp"
#include "hw_turjabot/servo.hpp"

namespace hw_turjabot
{
class TurjabotHardware : public hardware_interface::SystemInterface
{
  
  struct Config {
    std::string left_drive_wheel_name = "";
    std::string right_drive_wheel_name = "";
    std::string left_steer_wheel_name = "";
    std::string right_steer_wheel_name = "";
    std::string camera_pan_name = "";
    std::string camera_tilt_name = "";
    float loop_rate = 0.0;
    int enc_counts_per_rev = 0;
  };


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TurjabotHardware);

  HW_TURJABOT_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  HW_TURJABOT_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HW_TURJABOT_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HW_TURJABOT_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HW_TURJABOT_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HW_TURJABOT_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  HW_TURJABOT_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  TurjabotComms comms_;
  Config cfg_;

  Wheel wheel_dl_;
  Wheel wheel_dr_;

  Servo servo_l_steer_;
  Servo servo_r_steer_;
  Servo servo_cam_pan_;
  Servo servo_cam_tilt_;
};

}  // namespace hw_turjabot

#endif  // HW_TURJABOT__DIFFBOT_SYSTEM_HPP_
