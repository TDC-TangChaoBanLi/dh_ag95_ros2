#include "dh_ag95_controllers/dh_ag95_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dh_ag95_controllers {
namespace {
dh_ag95::TransportType transport_from_string(const std::string& s) {
  if (s == "official_serial") return dh_ag95::TransportType::OfficialSerial;
  if (s == "socketcan") return dh_ag95::TransportType::SocketCan;
  if (s == "pcanbasic") return dh_ag95::TransportType::PcanBasic;
  if (s == "modbus_rtu") return dh_ag95::TransportType::ModbusRtu;
  throw std::invalid_argument("unknown transport_type: " + s);
}
}

DhAg95Hardware::~DhAg95Hardware() {
  polling_running_.store(false, std::memory_order_relaxed);
  if (polling_thread_.joinable()) polling_thread_.join();
}

hardware_interface::CallbackReturn DhAg95Hardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  joint_name_ = param("joint_name", info_.joints.empty() ? "dh_ag95_finger_joint" : info_.joints[0].name);
  model_params_ = dh_ag95::get_gripper_params(param("gripper_model", "ag-160-95"));
  write_deadband_ = param_double("write_deadband", 0.005);
  default_force_percent_ = param_int("default_force_percent", 30);
  polling_interval_ms_ = param_int("polling_interval_ms", 50);
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
  cfg.gripper_model = param("gripper_model", "ag-160-95");
  cfg.default_force_percent = default_force_percent_;
  cfg.max_retries = param_int("max_retries", 2);
  cfg.wait_write_echo = param_bool("wait_write_echo", false);
  cfg.skip_duplicate_writes = param_bool("skip_duplicate_writes", true);
  cfg.official_serial.port = param("serial_port", "/dev/ttyACM0");
  cfg.official_serial.baudrate = param_int("serial_baudrate", 115200);
  cfg.socketcan.interface_name = param("can_interface", "can0");
  cfg.socketcan.bitrate = param_int("can_bitrate", 500000);
  cfg.pcanbasic.channel = param("pcan_channel", "PCAN_USBBUS1");
  cfg.pcanbasic.bitrate = param_int("pcan_bitrate", param_int("can_bitrate", 500000));
  cfg.modbus_rtu.port = param("serial_port", "/dev/ttyUSB0");
  cfg.modbus_rtu.baudrate = param_int("serial_baudrate", 115200);
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
    int pos = gripper_->get_position();
    hw_position_ = dh_ag95::position_percent_to_radians(pos < 0 ? 0.0 : pos, model_params_);
    cached_position_.store(hw_position_, std::memory_order_relaxed);
    cached_effort_.store(hw_effort_, std::memory_order_relaxed);
    cmd_position_ = hw_position_;

    // Start background polling thread.
    polling_running_.store(true, std::memory_order_relaxed);
    polling_thread_ = std::thread(&DhAg95Hardware::polling_loop, this);

    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("DhAg95Hardware"), "activate failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn DhAg95Hardware::on_deactivate(const rclcpp_lifecycle::State&) {
  // Stop polling thread first, then tear down hardware.
  polling_running_.store(false, std::memory_order_relaxed);
  if (polling_thread_.joinable()) polling_thread_.join();
  try { gripper_->disconnect(); } catch (...) {}
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DhAg95Hardware::read(const rclcpp::Time&, const rclcpp::Duration&) {
  // Lock-free: atomic cache populated by background polling_loop().
  double old = hw_position_;
  hw_position_ = cached_position_.load(std::memory_order_relaxed);
  hw_velocity_ = hw_position_ - old;
  hw_effort_ = cached_effort_.load(std::memory_order_relaxed);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DhAg95Hardware::write(const rclcpp::Time&, const rclcpp::Duration&) {
  int force_pct = static_cast<int>(std::round(
      dh_ag95::force_newtons_to_percent(cmd_effort_, model_params_)));
  int percent = static_cast<int>(std::round(
      dh_ag95::position_radians_to_percent(cmd_position_, model_params_)));
  bool pos_changed = std::abs(cmd_position_ - last_cmd_position_) > write_deadband_;
  bool force_changed = std::abs(cmd_effort_ - last_cmd_effort_) > 0.5;

  if (!pos_changed && !force_changed) {
    return hardware_interface::return_type::OK;
  }

  // try_lock: if the polling thread holds gripper_mutex_, skip this cycle.
  // The write will be retried next cycle when the values haven't changed yet.
  std::unique_lock<std::mutex> lock(gripper_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return hardware_interface::return_type::OK;
  }

  try {
    if (force_changed && force_pct >= 20) {
      gripper_->set_force(force_pct);
      last_cmd_effort_ = cmd_effort_;
    }
    if (pos_changed) {
      gripper_->set_position(percent);
      last_cmd_position_ = cmd_position_;
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("DhAg95Hardware"), "write failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

void DhAg95Hardware::polling_loop() {
  while (polling_running_.load(std::memory_order_relaxed)) {
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      try {
        int pos_pct = gripper_->get_position();
        int force_pct = gripper_->get_force();

        cached_position_.store(
            dh_ag95::position_percent_to_radians(
                pos_pct < 0 ? 0.0 : pos_pct, model_params_),
            std::memory_order_relaxed);
        cached_effort_.store(
            dh_ag95::force_percent_to_newtons(
                static_cast<double>(force_pct < 0 ? 0 : force_pct), model_params_),
            std::memory_order_relaxed);
      } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("DhAg95Hardware"),
                    "polling read failed: %s", e.what());
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(polling_interval_ms_));
  }
}

}  // namespace dh_ag95_controllers

PLUGINLIB_EXPORT_CLASS(dh_ag95_controllers::DhAg95Hardware, hardware_interface::ActuatorInterface)
