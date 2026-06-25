#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dh_ag95/ag95_gripper.hpp"
#include "dh_ag95/gripper_model.hpp"

namespace dh_ag95_controllers {

/// Atomic pending command posted by write() and consumed by polling_loop().
struct PendingWrite {
  std::atomic<bool> dirty{false};
  std::atomic<int> position_percent{-1};  ///< -1 = no pending position write, 0..100 = target
  std::atomic<int> force_percent{-1};     ///< -1 = no pending force write,    20..100 = target
};

class DhAg95Hardware : public hardware_interface::ActuatorInterface {
 public:
  ~DhAg95Hardware() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  dh_ag95::Ag95Config make_config_from_info();
  std::string param(const std::string& key, const std::string& fallback = "") const;
  double param_double(const std::string& key, double fallback) const;
  int param_int(const std::string& key, int fallback) const;
  bool param_bool(const std::string& key, bool fallback) const;

  /// Background polling loop: processes pending writes, then reads
  /// position + force from hardware, stores results in atomic caches.
  /// This is the ONLY thread that accesses gripper_ after activation.
  void polling_loop();

  std::unique_ptr<dh_ag95::Ag95Gripper> gripper_;
  dh_ag95::GripperModelParams model_params_;
  std::string joint_name_{"dh_ag95_finger_joint"};
  double write_deadband_{0.005};
  int default_force_percent_{30};
  int polling_interval_ms_{50};

  double hw_position_{0.0};
  double hw_velocity_{0.0};
  double hw_effort_{0.0};
  double cmd_position_{0.0};
  double cmd_effort_{30.0};
  double last_cmd_position_{-1.0};
  double last_cmd_effort_{-1.0};

  /// Atomic cache populated by polling_loop(), read lock-free by read().
  std::atomic<double> cached_position_{0.0};
  std::atomic<double> cached_effort_{0.0};

  /// Atomic pending command posted by write(), consumed by polling_loop().
  PendingWrite pending_write_;

  /// Background polling thread.  Started in on_activate(), joined in on_deactivate().
  std::thread polling_thread_;
  std::atomic<bool> polling_running_{false};
};

}  // namespace dh_ag95_controllers
