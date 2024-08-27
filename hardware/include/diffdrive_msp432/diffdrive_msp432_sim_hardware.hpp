#ifndef DIFFDRIVE_MSP432_SIM_HARDWARE_HPP_
#define DIFFDRIVE_MSP432_SIM_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "diffdrive_msp432/visibility_control.h"
#include "diffdrive_msp432/wheel.hpp"

namespace diffdrive_msp432
{
class DiffDriveMSP432SimHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveMSP432SimHardware)

  DIFFDRIVE_MSP432_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_MSP432_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_MSP432_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_MSP432_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_MSP432_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_MSP432_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_MSP432_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_MSP432_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_MSP432_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Wheel wheel_l_;
  Wheel wheel_r_;
};

}  // namespace diffdrive_msp432

#endif  // DIFFDRIVE_MSP432_SIM_HARDWARE_HPP_
