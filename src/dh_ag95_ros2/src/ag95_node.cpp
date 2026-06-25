#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/gripper_command.hpp>

#include "dh_ag95/ag95_gripper.hpp"
#include "dh_ag95/gripper_model.hpp"
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

/// 拼接 namespace 与名称；如果 name 已经是绝对路径（以 / 开头）则直接返回 name。
std::string join_name(const std::string& prefix, const std::string& name) {
  if (!name.empty() && name.front() == '/') return name;
  if (prefix.empty()) return name;
  if (prefix.back() == '/') return prefix + name;
  return prefix + "/" + name;
}

/// 把字符串形式的 transport_type 转换为枚举，不识别则抛出异常。
dh_ag95::TransportType transport_from_string(const std::string& s) {
  if (s == "official_serial") return dh_ag95::TransportType::OfficialSerial;
  if (s == "socketcan") return dh_ag95::TransportType::SocketCan;
  if (s == "pcanbasic") return dh_ag95::TransportType::PcanBasic;
  if (s == "modbus_rtu") return dh_ag95::TransportType::ModbusRtu;
  throw std::invalid_argument("unknown transport_type: " + s);
}
}  // namespace

class Ag95Node : public rclcpp::Node {
 public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperCommand>;

  explicit Ag95Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("dh_ag95_node", options) {
    declare_parameters();
    auto cfg = make_config();
    model_params_ = dh_ag95::get_gripper_params(cfg.gripper_model);
    gripper_ = std::make_unique<dh_ag95::Ag95Gripper>(cfg);

    namespace_ = get_parameter("namespace").as_string();

    state_pub_topic_name_   = declare_or_get_name("state_pub_topic_name",   join_name(namespace_, "state"));
    command_sub_topic_name_ = declare_or_get_name("command_sub_topic_name", join_name(namespace_, "command"));
    drop_pub_topic_name_    = declare_or_get_name("drop_pub_topic_name",    join_name(namespace_, "drop_event"));
    command_action_name_    = declare_or_get_name("command_action_name",    join_name(namespace_, "move"));
    command_service_prefix_name_ = declare_or_get_name("command_service_prefix_name", "");

    // 按开关选择性地创建订阅/服务/action；发布器始终创建
    if (get_parameter("is_launch_command_service").as_bool()) create_service_servers();
    if (get_parameter("is_launch_command_action").as_bool())  create_action_servers();
    if (get_parameter("is_launch_command_topic").as_bool())   create_sub_topics();
    create_pub_topics();

    try {
      gripper_->connect();
      RCLCPP_INFO(get_logger(), "Connected AG-95 using transport '%s'",
                  this->get_parameter("transport_type").as_string().c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to connect AG-95: %s", e.what());
    }

    if (this->get_parameter("auto_initialize").as_bool()) {
      try {
        gripper_->initialize(true);
        cached_initialized_.store(true, std::memory_order_release);
        RCLCPP_INFO(get_logger(), "AG-95 initialized successfully. Now ready for commands.");
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(),
            "AG-95 auto-initialize issue: %s. Node will continue running; call 'initialize' service to retry.",
            e.what());
      }
    }

    // feedback_timer_: 驱动 drop_event 轮询 + state 发布（频率由 feedback_rate_hz 控制）
    double feedback_rate = get_parameter("feedback_rate_hz").as_double();
    if (feedback_rate <= 0.0) feedback_rate = 20.0;
    auto feedback_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / feedback_rate));
    feedback_timer_ = create_wall_timer(feedback_period, [this]() {
      poll_event();
      publish_state();
    });

    // command_timer_: 按 command_rate_hz 将最近一条 /command 下发到硬件
    int command_interval_ms = get_parameter("command_interval_ms").as_int();
    if (command_interval_ms < 1) command_interval_ms = 30;
    const double hardware_max_hz = 1000.0 / static_cast<double>(command_interval_ms);
    double command_rate = get_parameter("command_rate_hz").as_double();
    if (command_rate <= 0.0) command_rate = 20.0;
    if (command_rate > hardware_max_hz) {
      RCLCPP_WARN(get_logger(),
          "command_rate_hz(%.1f) exceeds hardware max (command_interval_ms=%d -> %.1f Hz); clamped.",
          command_rate, command_interval_ms, hardware_max_hz);
      command_rate = hardware_max_hz;
    }
    auto command_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / command_rate));
    RCLCPP_INFO(get_logger(),
        "feedback_rate=%.2f Hz, command_rate=%.2f Hz (clamped by command_interval_ms=%d)",
        feedback_rate, command_rate, command_interval_ms);
    command_timer_ = create_wall_timer(command_period, [this]() { drain_command(); });
    RCLCPP_INFO(get_logger(), "Node initialized. Now ready for commands.");

    action_timeout_sec_ = get_parameter("action_timeout_sec").as_double();
  }

 private:
  // ---- 成员变量 -----------------------------------------------------------
  std::unique_ptr<dh_ag95::Ag95Gripper> gripper_;
  dh_ag95::GripperModelParams model_params_;      // 单位转换参数（来自驱动）
  double action_timeout_sec_ = 3.0;               // action 默认超时
  std::string namespace_;
  std::string state_pub_topic_name_, command_sub_topic_name_, drop_pub_topic_name_;
  std::string command_service_prefix_name_;
  std::string command_action_name_;
  rclcpp::Publisher<dh_ag95_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<dh_ag95_msgs::msg::DropEvent>::SharedPtr drop_pub_;
  rclcpp::Subscription<control_msgs::msg::GripperCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_, command_timer_;
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;

  // /command topic 节流：订阅回调写入原子变量，command_timer_ 读出并下发硬件。
  std::atomic<int> pending_position_{-1};
  std::atomic<int> pending_force_{-1};
  std::atomic<bool> has_pending_command_{false};

  // initialized 状态缓存：由 initialize service / auto_initialize 更新，
  // build_state_msg() 直接使用缓存值避免 is_initialized() 的额外 CAN 事务。
  std::atomic<bool> cached_initialized_{false};

  // ---- Action 异步执行（定时器轮询，不阻塞 executor）------------------------
  struct ActiveGoal {
    std::shared_ptr<GoalHandleGripper> goal_handle;
    rclcpp::Time deadline;
    bool cancel_requested = false;
  };
  std::optional<ActiveGoal> active_goal_;
  rclcpp::TimerBase::SharedPtr action_timer_;

  // =====================================================================
  //  参数声明与配置
  // =====================================================================

  /// 声明此节点用到的所有 ROS 参数。
  void declare_parameters() {
    // --- 名称类 ---
    declare_parameter<std::string>("namespace", "");
    declare_parameter<std::string>("state_pub_topic_name", "");
    declare_parameter<std::string>("drop_pub_topic_name", "");
    declare_parameter<std::string>("command_sub_topic_name", "");
    declare_parameter<std::string>("command_service_prefix_name", "");
    declare_parameter<std::string>("command_action_name", "");

    // --- 接口开关 ---
    declare_parameter<bool>("is_launch_command_topic", false);
    declare_parameter<bool>("is_launch_command_service", true);
    declare_parameter<bool>("is_launch_command_action", true);

    // --- 硬件与通信 ---
    declare_parameter<int>("gripper_id", 1);
    declare_parameter<std::string>("transport_type", "official_serial");
    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int>("serial_baudrate", 115200);
    declare_parameter<std::string>("can_interface", "can0");
    declare_parameter<int>("can_bitrate", 500000);
    declare_parameter<std::string>("pcan_channel", "PCAN_USBBUS1");
    declare_parameter<int>("pcan_bitrate", 500000);
    declare_parameter<int>("command_interval_ms", 30);
    declare_parameter<int>("read_timeout_ms", 200);
    declare_parameter<bool>("auto_initialize", false);
    declare_parameter<int>("default_force_percent", 30);
    declare_parameter<double>("feedback_rate_hz", 20.0);
    declare_parameter<double>("command_rate_hz", 20.0);

    // --- Action 配置 ---
    declare_parameter<double>("action_timeout_sec", 3.0);

    // --- 驱动高级配置 ---
    declare_parameter<int>("max_retries", 2);
    declare_parameter<bool>("wait_write_echo", true);
    declare_parameter<bool>("skip_duplicate_writes", true);
  }

  /// 若 param 已设置且非空则返回其值，否则返回 fallback 作为默认名。
  std::string declare_or_get_name(const std::string& param, const std::string& fallback) {
    auto v = get_parameter(param).as_string();
    return v.empty() ? fallback : v;
  }

  /// 把 ROS 参数汇总到底层驱动的配置结构体。
  dh_ag95::Ag95Config make_config() {
    dh_ag95::Ag95Config cfg;
    cfg.gripper_id         = static_cast<uint8_t>(get_parameter("gripper_id").as_int());
    cfg.transport_type     = transport_from_string(get_parameter("transport_type").as_string());
    cfg.command_interval   = std::chrono::milliseconds(get_parameter("command_interval_ms").as_int());
    cfg.read_timeout       = std::chrono::milliseconds(get_parameter("read_timeout_ms").as_int());
    cfg.auto_initialize    = false;  // 节点层自己控制，避免驱动内部再做一次
    cfg.default_force_percent = get_parameter("default_force_percent").as_int();
    cfg.max_retries        = get_parameter("max_retries").as_int();
    cfg.wait_write_echo    = get_parameter("wait_write_echo").as_bool();
    cfg.skip_duplicate_writes = get_parameter("skip_duplicate_writes").as_bool();

    cfg.official_serial.port     = get_parameter("serial_port").as_string();
    cfg.official_serial.baudrate = get_parameter("serial_baudrate").as_int();
    cfg.socketcan.interface_name = get_parameter("can_interface").as_string();
    cfg.socketcan.bitrate        = get_parameter("can_bitrate").as_int();
    cfg.pcanbasic.channel        = get_parameter("pcan_channel").as_string();
    cfg.pcanbasic.bitrate        = get_parameter("pcan_bitrate").as_int();
    cfg.modbus_rtu.port          = get_parameter("serial_port").as_string();
    cfg.modbus_rtu.baudrate      = get_parameter("serial_baudrate").as_int();
    return cfg;
  }

  // =====================================================================
  //  状态读取 —— 按场景选择合适的方法
  // =====================================================================

  /// 从 Ag95State 结构体转换为 ROS State 消息（用于 get_all_state() 一次性读取后）。
  dh_ag95_msgs::msg::State to_msg(const dh_ag95::Ag95State& s) {
    dh_ag95_msgs::msg::State m;
    m.header.stamp           = now();
    m.position_percent       = s.position_percent;
    m.force_internal_percent = s.force_internal_percent;
    m.force_external_percent = s.force_external_percent;
    m.raw_status             = s.raw_status;
    m.initialized            = s.initialized;
    m.moving                 = s.moving;
    m.reached                = s.reached;
    m.grasped                = s.grasped;
    m.status_text            = dh_ag95::status_to_string(s.status);
    return m;
  }

  /// 轻量级构造 State 消息：使用独立的 get_position / get_force / get_status
  /// 共计 4 次 CAN 事务（vs get_all_state 的 5 次），用于周期 publish_state()。
  dh_ag95_msgs::msg::State build_state_msg() {
    dh_ag95_msgs::msg::State m;
    m.header.stamp = now();

    int pos = gripper_->get_position();
    m.position_percent = static_cast<double>(pos < 0 ? 0 : pos);

    int fi = gripper_->get_force(dh_ag95::kSubForceInternal);
    m.force_internal_percent = static_cast<double>(fi < 0 ? 0 : fi);

    int fe = gripper_->get_force(dh_ag95::kSubForceExternal);
    m.force_external_percent = static_cast<double>(fe < 0 ? 0 : fe);

    auto status = gripper_->get_status();
    m.raw_status  = static_cast<int32_t>(status);
    m.status_text = dh_ag95::status_to_string(status);
    m.moving  = (status == dh_ag95::GripperStatus::MovingOrDefault);
    m.reached = (status == dh_ag95::GripperStatus::ReachedPosition);
    m.grasped = (status == dh_ag95::GripperStatus::GraspedObject);
    m.initialized = cached_initialized_.load(std::memory_order_acquire);

    return m;
  }

  /// 一次性读取完整状态（5 次 CAN 事务），用于 service 一次性响应。
  dh_ag95_msgs::msg::State read_full_state_msg() {
    return to_msg(gripper_->get_all_state());
  }

  // =====================================================================
  //  周期性任务
  // =====================================================================

  /// state topic 发布回调：被 feedback_timer_ 周期性调用。
  void publish_state() {
    try { state_pub_->publish(build_state_msg()); }
    catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "state publish failed: %s", e.what());
    }
  }

  /// drop_event 轮询回调：非阻塞读取事件队列，仅 kFuncObjectDropped / kSubDropFeedback
  /// 事件帧转换为 ROS DropEvent 发布，其余事件忽略。
  void poll_event() {
    try {
      auto ev = gripper_->receive_event(0ms);
      if (ev && ev->function == dh_ag95::kFuncObjectDropped
            && ev->sub_function == dh_ag95::kSubDropFeedback) {
        dh_ag95_msgs::msg::DropEvent msg;
        msg.header.stamp = now();
        msg.gripper_id   = ev->id;
        msg.dropped      = true;
        msg.raw_value    = ev->value;
        drop_pub_->publish(msg);
      }
    } catch (...) {}
  }

  // =====================================================================
  //  阻塞等待（service 用）
  // =====================================================================

  /// 轮询硬件状态，直到 reached / grasped 或超时。
  /** 使用 get_status()（1 次 CAN 事务/轮询）替代 get_all_state()（5 次）。
   *  注意：此函数在 service 回调内阻塞（每 50ms 读一次），会阻塞 executor
   *  处理其他请求。长耗时场景请使用 action。*/
  dh_ag95_msgs::msg::State wait_for_done(double timeout_sec) {
    auto deadline = now() + rclcpp::Duration::from_seconds(timeout_sec);
    while (rclcpp::ok() && now() < deadline) {
      auto status = gripper_->get_status();
      if (status == dh_ag95::GripperStatus::ReachedPosition ||
          status == dh_ag95::GripperStatus::GraspedObject) {
        return build_state_msg();
      }
      std::this_thread::sleep_for(50ms);
    }
    return build_state_msg();
  }

  // =====================================================================
  //  Service 服务器
  // =====================================================================

  void create_service_servers() {
    // initialize：请求抓手做一次归零初始化
    services_.push_back(create_service<dh_ag95_msgs::srv::Initialize>(
      join_name(namespace_, command_service_prefix_name_ + "initialize"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::Initialize::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::Initialize::Response> resp) {
        try {
          gripper_->initialize(req->wait,
              std::chrono::milliseconds(static_cast<int>(
                  (req->timeout_sec > 0 ? req->timeout_sec : 3.0) * 1000)));
          cached_initialized_.store(true, std::memory_order_release);
          resp->success = true; resp->message = "ok"; resp->state = read_full_state_msg();
        } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // set_position：设置目标张开/闭合百分比；可选等待到达或抓取成功
    services_.push_back(create_service<dh_ag95_msgs::srv::SetPosition>(
      join_name(namespace_, command_service_prefix_name_ + "set_position"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::SetPosition::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::SetPosition::Response> resp) {
        try {
          gripper_->set_position(static_cast<int>(std::round(req->position_percent)));
          resp->state = req->wait
              ? wait_for_done(req->timeout_sec > 0 ? req->timeout_sec : 3.0)
              : build_state_msg();
          resp->success = true; resp->message = "ok";
        } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // set_force：设置夹持力百分比，sub_function=0 时默认使用内力寄存器
    services_.push_back(create_service<dh_ag95_msgs::srv::SetForce>(
      join_name(namespace_, command_service_prefix_name_ + "set_force"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::SetForce::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::SetForce::Response> resp) {
        try {
          uint8_t sub = (req->sub_function == 0) ? dh_ag95::kSubForceInternal : req->sub_function;
          gripper_->set_force(static_cast<int>(std::round(req->force_percent)), sub);
          resp->state = build_state_msg(); resp->success = true; resp->message = "ok";
        } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // get_state：主动读一次当前状态
    services_.push_back(create_service<dh_ag95_msgs::srv::GetState>(
      join_name(namespace_, command_service_prefix_name_ + "get_state"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::GetState::Request>,
             const std::shared_ptr<dh_ag95_msgs::srv::GetState::Response> resp) {
        try { resp->state = read_full_state_msg(); resp->success = true; resp->message = "ok"; }
        catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // get_firmware_version：读取固件版本号
    services_.push_back(create_service<dh_ag95_msgs::srv::GetFirmwareVersion>(
      join_name(namespace_, command_service_prefix_name_ + "get_firmware_version"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::GetFirmwareVersion::Request>,
             const std::shared_ptr<dh_ag95_msgs::srv::GetFirmwareVersion::Response> resp) {
        try {
          auto v = gripper_->get_firmware_version();
          resp->success = true; resp->message = "ok";
          resp->raw = v.raw; resp->b0 = v.b0; resp->b1 = v.b1;
          resp->b2 = v.b2;  resp->b3 = v.b3; resp->version_string = v.to_string();
        } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // set_can_id / set_can_baudrate：改变抓手的 CAN 地址或波特率（需重启生效）
    services_.push_back(create_service<dh_ag95_msgs::srv::SetCanId>(
      join_name(namespace_, command_service_prefix_name_ + "set_can_id"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::SetCanId::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::SetCanId::Response> resp) {
        try {
          if (!req->confirm) throw std::runtime_error("confirm must be true because gripper reboot is required");
          gripper_->set_can_id(req->new_id, req->use_broadcast_id);
          resp->success = true; resp->message = "ok; reboot gripper required";
        } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetCanBaudrate>(
      join_name(namespace_, command_service_prefix_name_ + "set_can_baudrate"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::SetCanBaudrate::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::SetCanBaudrate::Response> resp) {
        try {
          if (!req->confirm) throw std::runtime_error("confirm must be true because gripper reboot is required");
          gripper_->set_can_baudrate(static_cast<dh_ag95::CanBaudRateIndex>(req->index));
          resp->success = true; resp->message = "ok; reboot gripper required";
        } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // set_io_mode / set_io_parameter：IO 模式及 IO 参数设置
    services_.push_back(create_service<dh_ag95_msgs::srv::SetIoMode>(
      join_name(namespace_, command_service_prefix_name_ + "set_io_mode"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::SetIoMode::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::SetIoMode::Response> resp) {
        try { gripper_->set_io_mode_enabled(req->enable); resp->success = true; resp->message = "ok"; }
        catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));
    services_.push_back(create_service<dh_ag95_msgs::srv::SetIoParameter>(
      join_name(namespace_, command_service_prefix_name_ + "set_io_parameter"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::SetIoParameter::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::SetIoParameter::Response> resp) {
        try { gripper_->set_io_parameter(req->sub_function, req->value); resp->success = true; resp->message = "ok"; }
        catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // set_drop_detection：开关掉件检测功能
    services_.push_back(create_service<dh_ag95_msgs::srv::SetDropDetection>(
      join_name(namespace_, command_service_prefix_name_ + "set_drop_detection"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::SetDropDetection::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::SetDropDetection::Response> resp) {
        try { gripper_->set_drop_detection_enabled(req->enable); resp->success = true; resp->message = "ok"; }
        catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));

    // raw_register：读写任意寄存器，用于调试 / 高级场景
    services_.push_back(create_service<dh_ag95_msgs::srv::RawRegister>(
      join_name(namespace_, command_service_prefix_name_ + "raw_register"),
      [this](const std::shared_ptr<dh_ag95_msgs::srv::RawRegister::Request> req,
             const std::shared_ptr<dh_ag95_msgs::srv::RawRegister::Response> resp) {
        try {
          resp->value = req->write
              ? gripper_->write_register(req->function, req->sub_function, req->value)
              : gripper_->read_register (req->function, req->sub_function);
          resp->success = true; resp->message = "ok";
        } catch (const std::exception& e) { resp->success = false; resp->message = e.what(); }
      }));
  }

  // =====================================================================
  //  Action 服务器 — control_msgs::action::GripperCommand
  // =====================================================================

  void create_action_servers() {
    action_server_ = rclcpp_action::create_server<GripperCommand>(
      this, command_action_name_,
      std::bind(&Ag95Node::action_handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Ag95Node::action_handle_cancel,   this, std::placeholders::_1),
      std::bind(&Ag95Node::action_handle_accepted, this, std::placeholders::_1));
    action_timer_ = create_wall_timer(50ms, [this]() { action_tick(); });
  }

  /// 参数校验：position(rad) 必须在 [0, joint_angle_rad] 区间；已有活跃 goal 时 REJECT。
  rclcpp_action::GoalResponse action_handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const GripperCommand::Goal> goal) {
    const auto& cmd = goal->command;
    if (cmd.position < 0.0 || cmd.position > model_params_.joint_angle_rad)
      return rclcpp_action::GoalResponse::REJECT;
    if (active_goal_)
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /// 仅当 goal 是当前活跃 goal 时才接受取消。
  rclcpp_action::CancelResponse action_handle_cancel(
      const std::shared_ptr<GoalHandleGripper> goal_handle) {
    if (active_goal_ && active_goal_->goal_handle == goal_handle) {
      active_goal_->cancel_requested = true;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
  }

  /// 将标准 goal (m, N) 转换为硬件百分比，下发到 gripper，然后由 action_tick() 轮询。
  void action_handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle) {
    const auto& cmd = goal_handle->get_goal()->command;
    try {
      int pos_pct = static_cast<int>(std::round(
          dh_ag95::position_radians_to_percent(cmd.position, model_params_)));
      gripper_->set_position(pos_pct);

      if (cmd.max_effort > 0.0) {
        int force_pct = static_cast<int>(std::round(
            dh_ag95::force_newtons_to_percent(cmd.max_effort, model_params_)));
        if (force_pct >= 20) gripper_->set_force(force_pct);
      }

      active_goal_ = ActiveGoal{
        goal_handle,
        now() + rclcpp::Duration::from_seconds(action_timeout_sec_),
        false
      };
    } catch (const std::exception& e) {
      auto result = std::make_shared<GripperCommand::Result>();
      goal_handle->abort(result);
    }
  }

  /// 辅助：读取状态并填充 GripperCommand Result/Feedback 的 SI 单位字段。
  /// 使用模板以同时支持 Result 和 Feedback（两者字段相同但类型不同）。
  template <typename T>
  void fill_gripper_result(T& r) {
    try {
      auto status = gripper_->get_status();
      int pos_pct = gripper_->get_position();
      int eff_pct = gripper_->get_force(dh_ag95::kSubForceInternal);

      r.position = dh_ag95::position_percent_to_radians(
          static_cast<double>(pos_pct < 0 ? 0 : pos_pct), model_params_);
      r.effort = dh_ag95::force_percent_to_newtons(
          static_cast<double>(eff_pct < 0 ? 0 : eff_pct), model_params_);
      r.reached_goal = (status == dh_ag95::GripperStatus::ReachedPosition);
      r.stalled = (status == dh_ag95::GripperStatus::GraspedObject);
    } catch (...) {}
  }

  /// 50ms 定时器：检查 reached/stalled → succeed；canceled → canceled；timeout → abort。
  void action_tick() {
    if (!active_goal_) return;
    auto& ag = *active_goal_;

    if (ag.cancel_requested) {
      auto result = std::make_shared<GripperCommand::Result>();
      fill_gripper_result(*result);
      ag.goal_handle->canceled(result);
      active_goal_.reset();
      return;
    }

    try {
      auto status = gripper_->get_status();
      if (status == dh_ag95::GripperStatus::ReachedPosition ||
          status == dh_ag95::GripperStatus::GraspedObject) {
        auto result = std::make_shared<GripperCommand::Result>();
        fill_gripper_result(*result);
        ag.goal_handle->succeed(result);
        active_goal_.reset();
        return;
      }
    } catch (...) {}

    if (now() >= ag.deadline) {
      auto result = std::make_shared<GripperCommand::Result>();
      fill_gripper_result(*result);
      ag.goal_handle->abort(result);
      active_goal_.reset();
      return;
    }

    auto feedback = std::make_shared<GripperCommand::Feedback>();
    fill_gripper_result(*feedback);
    ag.goal_handle->publish_feedback(feedback);
  }

  // =====================================================================
  //  Topic 接口
  // =====================================================================

  void create_pub_topics() {
    state_pub_ = create_publisher<dh_ag95_msgs::msg::State>(state_pub_topic_name_, 10);
    drop_pub_  = create_publisher<dh_ag95_msgs::msg::DropEvent>(drop_pub_topic_name_, 10);
  }

  /// 创建 /command 订阅。队列长度 1——只关心"最新一条目标位置/力"。
  void create_sub_topics() {
    command_sub_ = create_subscription<control_msgs::msg::GripperCommand>(
        command_sub_topic_name_, 1,
        [this](control_msgs::msg::GripperCommand::SharedPtr msg) { topic_handle_command(*msg); });
  }

  /// /command topic 回调：将 SI 单位 (m, N) 转换为百分比后存入原子变量。
  /** 真正的下发由 command_timer_ 按 command_rate_hz 调用 drain_command() 完成。*/
  void topic_handle_command(const control_msgs::msg::GripperCommand& msg) {
    const int pos = static_cast<int>(std::round(
        dh_ag95::position_radians_to_percent(msg.position, model_params_)));
    const int frc = (msg.max_effort > 0.0)
        ? static_cast<int>(std::round(
            dh_ag95::force_newtons_to_percent(msg.max_effort, model_params_)))
        : -1;
    pending_position_.store(pos, std::memory_order_relaxed);
    pending_force_.store(frc, std::memory_order_relaxed);
    has_pending_command_.store(true, std::memory_order_release);
  }

  /// 由 command_timer_ 周期性调用：把最新一条 command 下发到硬件。
  void drain_command() {
    if (!has_pending_command_.exchange(false, std::memory_order_acq_rel)) return;
    const int pos = pending_position_.load(std::memory_order_relaxed);
    const int frc = pending_force_.load(std::memory_order_relaxed);
    try {
      if (frc >= 20) gripper_->set_force(frc);
      gripper_->set_position(pos);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "command drain failed: %s", e.what());
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ag95Node>());
  rclcpp::shutdown();
  return 0;
}
