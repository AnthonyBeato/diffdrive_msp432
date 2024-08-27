#include "diffdrive_msp432/diffdrive_msp432_sim_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace diffdrive_msp432
{
hardware_interface::CallbackReturn DiffDriveMSP432SimHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_l_.setup(info_.joints[0].name, std::stoi(info_.hardware_parameters["enc_counts_per_rev"]));
  wheel_r_.setup(info_.joints[1].name, std::stoi(info_.hardware_parameters["enc_counts_per_rev"]));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveMSP432SimHardware::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432SimHardware"), "Configuring simulated hardware...");

  // Aquí no necesitamos conectarnos a un hardware real, simplemente retornamos SUCCESS
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveMSP432SimHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveMSP432SimHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveMSP432SimHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432SimHardware"), "Cleaning up simulated hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveMSP432SimHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432SimHardware"), "Activating simulated hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveMSP432SimHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveMSP432SimHardware"), "Deactivating simulated hardware...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveMSP432SimHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // En la simulación simplemente incrementamos la posición basada en el comando y la velocidad
  double delta_seconds = period.seconds();

  wheel_l_.pos += wheel_l_.vel * delta_seconds;
  wheel_r_.pos += wheel_r_.vel * delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveMSP432SimHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Aquí simplemente ajustamos la velocidad a partir del comando actual
  wheel_l_.vel = wheel_l_.cmd;
  wheel_r_.vel = wheel_r_.cmd;

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_msp432

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diffdrive_msp432::DiffDriveMSP432SimHardware, hardware_interface::SystemInterface)
