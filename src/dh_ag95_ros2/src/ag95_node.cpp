#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "dh_ag95/ag95_gripper.hpp"
#include "dh_ag95_msgs/action/move_gripper.hpp"
#include "dh_ag95_msgs/msg/command.hpp"
#include "dh_ag95_msgs/msg/drop_event.hpp"
#include "dh_ag95_msgs/msg/state.hpp"
#include "dh_ag95_msgs/srv/get_firmware_version.hpp"
#include "dh_ag95_msgs/srv/get_state.hpp"
#include "dh_ag95_msgs/srv/initialize.hpp"
#include "dh_ag95_msgs/srv/raw_register.hpp"
#include "dh_ag95_msgs/srv/set_can_baudrate.hpp"
#include "dh_ag95_msgs/srv/set_can_id.hpp"
#include "dh_ag95_msgs/srv/set_drop_detection.hpp"
#include "dh_ag95_msgs/srv/set_force.hpp"
#include "dh_ag95_msgs/srv/set_io_mode.hpp"
#include "dh_ag95_msgs/srv/set_io_parameter.hpp"
#include "dh_ag95_msgs/srv/set_position.hpp"

using namespace std::chrono_literals;

namespace {
std::string join_name(const std::string& prefix, const std::string& name) {
  if (!name.empty() && name.front() == '/') return name;
  if (prefix.empty()) return name;
  if (prefix.back() == '/') return prefix + name;
  return prefix + "/" + name;
}

dh_ag95::TransportType transport_from_string(const std::string& s) {
  if (s == "official_serial") return dh_ag95::TransportType::OfficialSerial;
  if (s == "socketcan") return dh_ag95::TransportType::SocketCan;
  if (s == "slcan") return dh_ag95::TransportType::Slcan;
  if (s == "pcanbasic") return dh_ag95::TransportType::PcanBasic;
  if (s == "modbus_rtu") return dh_ag95::TransportType::ModbusRtu;
  throw std::invalid_argument("unknown transport_type: " + s);
}
}

class Ag95Node : public rclcpp::Node {
 public:
  using MoveGripper = dh_ag95_msgs::action::MoveGripper;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<MoveGripper>;

  explicit Ag95Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("dh_ag95_node", options) {
    declare_parameters();
    auto cfg = make_config();
    gripper_ = std::make_unique<dh_ag95::Ag95Gripper>(cfg);

    state_topic_ = declare_or_get_name("state_topic", join_name(topic_prefix_, "state"));
    command_topic_ = declare_or_get_name("command_topic", join_name(topic_prefix_, "command"));
    drop_event_topic_ = declare_or_get_name("drop_event_topic", join_name(topic_prefix_, "drop_event"));
    action_name_ = declare_or_get_name("action_name", join_name(topic_prefix_, "move"));
    service_prefix_ = this->get_parameter("service_prefix").as_string();
    if (service_prefix_.empty()) service_prefix_ = topic_prefix_;

    state_pub_ = create_publisher<dh_ag95_msgs::msg::State>(state_topic_, 10);
    drop_pub_ = create_publisher<dh_ag95_msgs::msg::DropEvent>(drop_event_topic_, 10);
    command_sub_ = create_subscription<dh_ag95_msgs::msg::Command>(command_topic_, 10, [this](dh_ag95_msgs::msg::Command::SharedPtr msg) {
      handle_command(*msg);
    });

    create_services();
    action_server_ = rclcpp_action::create_server<MoveGripper>(
      this, action_name_,
      std::bind(&Ag95Node::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Ag95Node::handle_cancel, this, std::placeholders::_1),
      std::bind(&Ag95Node::handle_accepted, this, std::placeholders::_1));

    try {
      gripper_->connect();
      RCLCPP_INFO(get_logger(), "Connected AG-95 using transport '%s'", this->get_parameter("transport_type").as_string().c_str());
      if (this->get_parameter("auto_initialize").as_bool()) gripper_->initialize(true);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to connect AG-95: %s", e.what());
    }

    double rate = this->get_parameter("feedback_rate_hz").as_double();
    if (rate <= 0.0) rate = 20.0;
    state_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / rate)), [this]() { publish_state(); });
    event_timer_ = create_wall_timer(50ms, [this]() { poll_event(); });
  }

 private:
  std::unique_ptr<dh_ag95::Ag95Gripper> gripper_;
  std::string topic_prefix_, service_prefix_, state_topic_, command_topic_, drop_event_topic_, action_name_;
  rclcpp::Publisher<dh_ag95_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<dh_ag95_msgs::msg::DropEvent>::SharedPtr drop_pub_;
  rclcpp::Subscription<dh_ag95_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr state_timer_, event_timer_;
  rclcpp_action::Server<MoveGripper>::SharedPtr action_server_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  void declare_parameters() {
    declare_parameter<std::string>("topic_prefix", "dh_ag95");
    declare_parameter<std::string>("service_prefix", "");
    declare_parameter<std::string>("state_topic", "");
    declare_parameter<std::string>("command_topic", "");
    declare_parameter<std::string>("drop_event_topic", "");
    declare_parameter<std::string>("action_name", "");
    declare_parameter<int>("gripper_id", 1);
    declare_parameter<std::string>("transport_type", "official_serial");
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<std::string>("can_interface", "can0");
    declare_parameter<int>("can_bitrate", 500000);
    declare_parameter<std::string>("pcan_channel", "PCAN_USBBUS1");
    declare_parameter<int>("pcan_bitrate", 500000);
    declare_parameter<int>("command_interval_ms", 30);
    declare_parameter<int>("read_timeout_ms", 200);
    declare_parameter<bool>("auto_initialize", false);
    declare_parameter<int>("default_force_percent", 30);
    declare_parameter<double>("feedback_rate_hz", 20.0);
    topic_prefix_ = get_parameter("topic_prefix").as_string();
  }

  std::string declare_or_get_name(const std::string& param, const std::string& fallback) {
    auto v = get_parameter(param).as_string();
    return v.empty() ? fallback : v;
  }

  dh_ag95::Ag95Config make_config() {
    dh_ag95::Ag95Config cfg;
    cfg.gripper_id = static_cast<uint8_t>(get_parameter("gripper_id").as_int());
    cfg.transport_type = transport_from_string(get_parameter("transport_type").as_string());
    cfg.command_interval = std::chrono::milliseconds(get_parameter("command_interval_ms").as_int());
    cfg.read_timeout = std::chrono::milliseconds(get_parameter("read_timeout_ms").as_int());
    cfg.auto_initialize = false; // handled by node after connect
    cfg.default_force_percent = get_parameter("default_force_percent").as_int();
    cfg.official_serial.port = get_parameter("port").as_string();
    cfg.official_serial.baudrate = get_parameter("baudrate").as_int();
    cfg.slcan.port = get_parameter("port").as_string();
    cfg.slcan.serial_baudrate = get_parameter("baudrate").as_int();
    cfg.slcan.can_bitrate = get_parameter("can_bitrate").as_int();
    cfg.socketcan.interface_name = get_parameter("can_interface").as_string();
    cfg.socketcan.bitrate = get_parameter("can_bitrate").as_int();
    cfg.pcanbasic.channel = get_parameter("pcan_channel").as_string();
    cfg.pcanbasic.bitrate = get_parameter("pcan_bitrate").as_int();
    return cfg;
  }

  dh_ag95_msgs::msg::State to_msg(const dh_ag95::Ag95State& s) {
    dh_ag95_msgs::msg::State m;
    m.header.stamp = now();
    m.position_percent = s.position_percent;
    m.force_internal_percent = s.force_internal_percent;
    m.force_external_percent = s.force_external_percent;
    m.raw_status = s.raw_status;
    m.initialized = s.initialized;
    m.moving = s.moving;
    m.reached = s.reached;
    m.grasped = s.grasped;
    m.status_text = dh_ag95::status_to_string(s.status);
    return m;
  }

  dh_ag95_msgs::msg::State read_state_msg() {
    return to_msg(gripper_->read_state());
  }

  void publish_state() {
    try { state_pub_->publish(read_state_msg()); }
    catch (const std::exception& e) { RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "read state failed: %s", e.what()); }
  }

  void poll_event() {
    try {
      auto ev = gripper_->receive_event(0ms);
      if (ev && ev->function == dh_ag95::kFuncObjectDropped && ev->sub_function == dh_ag95::kSubDropFeedback) {
        dh_ag95_msgs::msg::DropEvent msg;
        msg.header.stamp = now(); msg.gripper_id = ev->id; msg.dropped = true; msg.raw_value = ev->value;
        drop_pub_->publish(msg);
      }
    } catch (...) {}
  }

  void handle_command(const dh_ag95_msgs::msg::Command& msg) {
    try {
      if (msg.force_percent > 0) gripper_->set_force(static_cast<int>(std::round(msg.force_percent)));
      gripper_->set_position(static_cast<int>(std::round(msg.position_percent)));
      if (msg.wait) wait_for_done(msg.timeout_sec > 0 ? msg.timeout_sec : 3.0);
    } catch (const std::exception& e) { RCLCPP_ERROR(get_logger(), "command failed: %s", e.what()); }
  }

  dh_ag95_msgs::msg::State wait_for_done(double timeout_sec) {
    auto deadline = now() + rclcpp::Duration::from_seconds(timeout_sec);
    dh_ag95_msgs::msg::State st;
    while (rclcpp::ok() && now() < deadline) {
      st = read_state_msg();
      if (st.reached || st.grasped) return st;
      std::this_thread::sleep_for(50ms);
    }
    return st;
  }

  void create_services() {
    services_.push_back(create_service<dh_ag95_msgs::srv::Initialize>(join_name(service_prefix_, "initialize"), [this](auto req, auto resp) {
      try { gripper_->initialize(req->wait, std::chrono::milliseconds(static_cast<int>((req->timeout_sec > 0 ? req->timeout_sec : 3.0) * 1000))); resp->success = true; resp->message = "ok"; resp->state = read_state_msg(); }
      catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
    }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetPosition>(join_name(service_prefix_, "set_position"), [this](auto req, auto resp) {
      try { gripper_->set_position(static_cast<int>(std::round(req->position_percent))); resp->state = req->wait ? wait_for_done(req->timeout_sec > 0 ? req->timeout_sec : 3.0) : read_state_msg(); resp->success = true; resp->message = "ok"; }
      catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
    }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetForce>(join_name(service_prefix_, "set_force"), [this](auto req, auto resp) {
      try { uint8_t sub = req->sub_function == 0 ? dh_ag95::kSubForceInternal : req->sub_function; gripper_->set_force(static_cast<int>(std::round(req->force_percent)), sub); resp->state = read_state_msg(); resp->success = true; resp->message = "ok"; }
      catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
    }));
    services_.push_back(create_service<dh_ag95_msgs::srv::GetState>(join_name(service_prefix_, "get_state"), [this](auto, auto resp) { try { resp->state = read_state_msg(); resp->success = true; resp->message = "ok"; } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
    services_.push_back(create_service<dh_ag95_msgs::srv::GetFirmwareVersion>(join_name(service_prefix_, "get_firmware_version"), [this](auto, auto resp) { try { auto v = gripper_->get_firmware_version(); resp->success = true; resp->message = "ok"; resp->raw = v.raw; resp->b0 = v.b0; resp->b1 = v.b1; resp->b2 = v.b2; resp->b3 = v.b3; resp->version_string = v.to_string(); } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetCanId>(join_name(service_prefix_, "set_can_id"), [this](auto req, auto resp) { try { if (!req->confirm) throw std::runtime_error("confirm must be true because gripper reboot is required"); gripper_->set_can_id(req->new_id, req->use_broadcast_id); resp->success = true; resp->message = "ok; reboot gripper required"; } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetCanBaudrate>(join_name(service_prefix_, "set_can_baudrate"), [this](auto req, auto resp) { try { if (!req->confirm) throw std::runtime_error("confirm must be true because gripper reboot is required"); gripper_->set_can_baudrate(static_cast<dh_ag95::CanBaudRateIndex>(req->index)); resp->success = true; resp->message = "ok; reboot gripper required"; } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetIoMode>(join_name(service_prefix_, "set_io_mode"), [this](auto req, auto resp) { try { gripper_->set_io_mode_enabled(req->enable); resp->success = true; resp->message = "ok"; } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetIoParameter>(join_name(service_prefix_, "set_io_parameter"), [this](auto req, auto resp) { try { gripper_->set_io_parameter(req->sub_function, req->value); resp->success = true; resp->message = "ok"; } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetDropDetection>(join_name(service_prefix_, "set_drop_detection"), [this](auto req, auto resp) { try { gripper_->set_drop_detection_enabled(req->enable); resp->success = true; resp->message = "ok"; } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
    services_.push_back(create_service<dh_ag95_msgs::srv::RawRegister>(join_name(service_prefix_, "raw_register"), [this](auto req, auto resp) { try { resp->value = req->write ? gripper_->write_register(req->function, req->sub_function, req->value) : gripper_->read_register(req->function, req->sub_function); resp->success = true; resp->message = "ok"; } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); } }));
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveGripper::Goal> goal) {
    if (goal->position_percent < 0.0 || goal->position_percent > 100.0) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove>) { return rclcpp_action::CancelResponse::ACCEPT; }
  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) { std::thread([this, goal_handle]() { execute(goal_handle); }).detach(); }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveGripper::Feedback>();
    auto result = std::make_shared<MoveGripper::Result>();
    try {
      if (goal->force_percent > 0) gripper_->set_force(static_cast<int>(std::round(goal->force_percent)));
      gripper_->set_position(static_cast<int>(std::round(goal->position_percent)));
      const double timeout = goal->timeout_sec > 0 ? goal->timeout_sec : 3.0;
      auto deadline = now() + rclcpp::Duration::from_seconds(timeout);
      while (rclcpp::ok() && now() < deadline) {
        if (goal_handle->is_canceling()) { result->success = false; result->message = "canceled"; result->state = read_state_msg(); goal_handle->canceled(result); return; }
        feedback->state = read_state_msg(); goal_handle->publish_feedback(feedback);
        if (feedback->state.reached || feedback->state.grasped) { result->success = true; result->message = "done"; result->state = feedback->state; goal_handle->succeed(result); return; }
        std::this_thread::sleep_for(50ms);
      }
      result->success = false; result->message = "timeout"; result->state = read_state_msg(); goal_handle->abort(result);
    } catch (const std::exception& e) { result->success = false; result->message = e.what(); try { result->state = read_state_msg(); } catch (...) {} goal_handle->abort(result); }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ag95Node>());
  rclcpp::shutdown();
  return 0;
}
