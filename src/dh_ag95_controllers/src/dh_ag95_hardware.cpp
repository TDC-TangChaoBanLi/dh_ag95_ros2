#include "dh_ag95_controllers/dh_ag95_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dh_ag95_controllers {
namespace {
dh_ag95::TransportType transport_from_string(const std::string& s) {
  if (s == "official_serial") return dh_ag95::TransportType::OfficialSerial;
  if (s == "socketcan") return dh_ag95::TransportType::SocketCan;
  if (s == "slcan") return dh_ag95::TransportType::Slcan;
  if (s == "pcanbasic") return dh_ag95::TransportType::PcanBasic;
  if (s == "modbus_rtu") return dh_ag95::TransportType::ModbusRtu;
  throw std::invalid_argument("unknown transport_type: " + s);
}
}

hardware_interface::CallbackReturn DhAg95Hardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  joint_name_ = param("joint_name", info_.joints.empty() ? "dh_ag95_finger_joint" : info_.joints[0].name);
  command_unit_ = param("command_unit", "normalized");
  stroke_m_ = param_double("stroke_m", 0.095);
  invert_position_ = param_bool("invert_position", false);
  write_deadband_ = param_double("write_deadband", 0.01);
  default_force_percent_ = param_int("default_force_percent", 30);
  cmd_effort_ = default_force_percent_;
  gripper_ = std::make_unique<dh_ag95::Ag95Gripper>(make_config_from_info());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::string DhAg95Hardware::param(const std::string& key, const std::string& fallback) const {
  auto it = info_.hardware_parameters.find(key);
  return it == info_.hardware_parameters.end() ? fallback : it->second;
}
int DhAg95Hardware::param_int(const std::string& key, int fallback) const { try { return std::stoi(param(key, std::to_string(fallback))); } catch (...) { return fallback; } }
double DhAg95Hardware::param_double(const std::string& key, double fallback) const { try { return std::stod(param(key, std::to_string(fallback))); } catch (...) { return fallback; } }
bool DhAg95Hardware::param_bool(const std::string& key, bool fallback) const { auto v = param(key, fallback ? "true" : "false"); return v == "true" || v == "1" || v == "yes"; }

dh_ag95::Ag95Config DhAg95Hardware::make_config_from_info() {
  dh_ag95::Ag95Config cfg;
  cfg.gripper_id = static_cast<uint8_t>(param_int("gripper_id", 1));
  cfg.transport_type = transport_from_string(param("transport_type", "official_serial"));
  cfg.command_interval = std::chrono::milliseconds(param_int("command_interval_ms", 30));
  cfg.read_timeout = std::chrono::milliseconds(param_int("read_timeout_ms", 200));
  cfg.auto_initialize = false;
  cfg.default_force_percent = default_force_percent_;
  cfg.official_serial.port = param("port", "/dev/ttyACM0");
  cfg.official_serial.baudrate = param_int("baudrate", 115200);
  cfg.slcan.port = param("port", "/dev/ttyUSB0");
  cfg.slcan.serial_baudrate = param_int("baudrate", 115200);
  cfg.slcan.can_bitrate = param_int("can_bitrate", 500000);
  cfg.socketcan.interface_name = param("can_interface", "can0");
  cfg.socketcan.bitrate = param_int("can_bitrate", 500000);
  cfg.pcanbasic.channel = param("pcan_channel", "PCAN_USBBUS1");
  cfg.pcanbasic.bitrate = param_int("pcan_bitrate", param_int("can_bitrate", 500000));
  return cfg;
}

std::vector<hardware_interface::StateInterface> DhAg95Hardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &hw_position_);
  out.emplace_back(joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_);
  out.emplace_back(joint_name_, hardware_interface::HW_IF_EFFORT, &hw_effort_);
  return out;
}

std::vector<hardware_interface::CommandInterface> DhAg95Hardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back(joint_name_, hardware_interface::HW_IF_POSITION, &cmd_position_);
  out.emplace_back(joint_name_, hardware_interface::HW_IF_EFFORT, &cmd_effort_);
  return out;
}

hardware_interface::CallbackReturn DhAg95Hardware::on_activate(const rclcpp_lifecycle::State&) {
  try {
    gripper_->connect();
    if (param_bool("auto_initialize", true)) gripper_->initialize(true);
    if (default_force_percent_ > 0) gripper_->set_force(default_force_percent_);
    auto st = gripper_->read_state();
    hw_position_ = percent_to_command_unit(st.position_percent < 0 ? 0.0 : st.position_percent);
    cmd_position_ = hw_position_;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DhAg95Hardware"), "activate failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn DhAg95Hardware::on_deactivate(const rclcpp_lifecycle::State&) {
  try { gripper_->disconnect(); } catch (...) {}
  return hardware_interface::CallbackReturn::SUCCESS;
}

double DhAg95Hardware::percent_to_command_unit(double percent) const {
  if (invert_position_) percent = 100.0 - percent;
  if (command_unit_ == "percent") return percent;
  if (command_unit_ == "meter") return percent / 100.0 * stroke_m_;
  return percent / 100.0;
}

int DhAg95Hardware::command_unit_to_percent(double command) const {
  double percent = 0.0;
  if (command_unit_ == "percent") percent = command;
  else if (command_unit_ == "meter") percent = command / stroke_m_ * 100.0;
  else percent = command * 100.0;
  if (invert_position_) percent = 100.0 - percent;
  percent = std::clamp(percent, 0.0, 100.0);
  return static_cast<int>(std::round(percent));
}

hardware_interface::return_type DhAg95Hardware::read(const rclcpp::Time&, const rclcpp::Duration&) {
  try {
    auto st = gripper_->read_state();
    double old = hw_position_;
    hw_position_ = percent_to_command_unit(st.position_percent < 0 ? 0.0 : st.position_percent);
    hw_velocity_ = hw_position_ - old;
    hw_effort_ = st.force_internal_percent;
    return hardware_interface::return_type::OK;
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("DhAg95Hardware"), "read failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type DhAg95Hardware::write(const rclcpp::Time&, const rclcpp::Duration&) {
  try {
    int force = static_cast<int>(std::round(std::clamp(cmd_effort_, 20.0, 100.0)));
    int percent = command_unit_to_percent(cmd_position_);
    if (std::abs(cmd_position_ - last_cmd_position_) > write_deadband_) {
      gripper_->set_force(force);
      gripper_->set_position(percent);
      last_cmd_position_ = cmd_position_;
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("DhAg95Hardware"), "write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace dh_ag95_controllers

PLUGINLIB_EXPORT_CLASS(dh_ag95_controllers::DhAg95Hardware, hardware_interface::ActuatorInterface)
