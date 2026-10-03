// Bridge between the Unitree G1 low-level DDS interface (unitree_hg) and
// generic ROS 2 messages. Bridged joint i is motor slot motor_indices[i]
// (default: i), so variants with missing motors (e.g. the 23-DoF G1, without
// waist roll/pitch and wrist pitch/yaw) map onto the 35-slot LowCmd/LowState.
//
//   /lowstate (unitree_hg/LowState) -> /imu (sensor_msgs/Imu)
//                                    -> /joint_states (sensor_msgs/JointState)
//   /getup/joint_command (sensor_msgs/JointState, positions) -> /lowcmd
//
// LowCmd is streamed at a fixed rate with per-joint Kp/Kd from parameters.
// If no fresh joint command is available the motors are put in damping mode
// (kp = 0, kd = damping_kd). Nothing is sent on /lowcmd unless the
// `enable_lowcmd` parameter is true (can be toggled at runtime with
// `ros2 param set`). The robot must be in debug mode (high-level motion
// control released) before enabling it.
//
// Emergency stop: a latched e-stop (~/estop service, /getup/estop topic or
// loss of the GUI heartbeat on /getup/heartbeat) forces damping regardless of
// incoming commands until ~/reset_estop is called. Gains, limits and timeouts
// can be changed at runtime through ROS parameters.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "getup/motor_crc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace getup
{

using SteadyClock = std::chrono::steady_clock;

class G1LowLevelBridge : public rclcpp::Node
{
public:
  G1LowLevelBridge()
  : Node("g1_lowlevel_bridge")
  {
    // joint_names[i] <-> motor slot motor_indices[i] (default: i).
    joint_names_ =
      declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    const auto motor_indices =
      declare_parameter<std::vector<int64_t>>("motor_indices", std::vector<int64_t>{});
    kp_ = declare_parameter<std::vector<double>>("kp", std::vector<double>{});
    kd_ = declare_parameter<std::vector<double>>("kd", std::vector<double>{});
    damping_kd_ = declare_parameter<double>("damping_kd", 1.0);
    pos_lower_ =
      declare_parameter<std::vector<double>>("position_lower", std::vector<double>{});
    pos_upper_ =
      declare_parameter<std::vector<double>>("position_upper", std::vector<double>{});
    max_target_delta_ = declare_parameter<double>("max_target_delta", 0.0);
    mode_pr_ = static_cast<uint8_t>(declare_parameter<int>("mode_pr", 0));
    enable_lowcmd_ = declare_parameter<bool>("enable_lowcmd", false);
    command_timeout_ = declare_parameter<double>("command_timeout_s", 0.1);
    heartbeat_timeout_ = declare_parameter<double>("heartbeat_timeout_s", 0.5);
    const auto lowcmd_rate_hz = declare_parameter<double>("lowcmd_rate_hz", 500.0);
    const auto lowstate_topic = declare_parameter<std::string>("lowstate_topic", "/lowstate");
    const auto lowcmd_topic = declare_parameter<std::string>("lowcmd_topic", "/lowcmd");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const auto joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/getup/joint_command");
    const auto estop_topic = declare_parameter<std::string>("estop_topic", "/getup/estop");
    const auto heartbeat_topic =
      declare_parameter<std::string>("heartbeat_topic", "/getup/heartbeat");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_in_pelvis");

    const size_t n = joint_names_.size();
    if (n == 0 || n > kNumMotorSlots) {
      throw std::invalid_argument("joint_names must have 1..35 entries");
    }
    if (motor_indices.empty()) {
      for (size_t i = 0; i < n; ++i) {
        motor_index_.push_back(i);
      }
    } else {
      std::set<int64_t> unique(motor_indices.begin(), motor_indices.end());
      if (motor_indices.size() != n || unique.size() != n || *unique.begin() < 0 ||
        *unique.rbegin() >= static_cast<int64_t>(kNumMotorSlots))
      {
        throw std::invalid_argument(
                "motor_indices must have len(joint_names) unique entries in [0, 35)");
      }
      motor_index_.assign(motor_indices.begin(), motor_indices.end());
    }
    std::string error = validate_gains_and_limits(kp_, kd_, pos_lower_, pos_upper_);
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    for (size_t i = 0; i < n; ++i) {
      name_to_motor_[joint_names_[i]] = i;
    }

    q_.assign(n, 0.0);
    target_.assign(n, 0.0);
    joint_state_msg_.name = joint_names_;
    joint_state_msg_.position.assign(n, 0.0);
    joint_state_msg_.velocity.assign(n, 0.0);
    joint_state_msg_.effort.assign(n, 0.0);
    imu_msg_.header.frame_id = imu_frame_id_;

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, rclcpp::SensorDataQoS());
    lowcmd_pub_ = create_publisher<unitree_hg::msg::LowCmd>(lowcmd_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    // Re-broadcast latches from any source (e.g. the dead-man) so the policy
    // node stops as well.
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(estop_topic, rclcpp::QoS(10).reliable());

    lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
      lowstate_topic, 10,
      [this](unitree_hg::msg::LowState::ConstSharedPtr msg) {on_lowstate(*msg);});
    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      command_topic, 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_command(*msg);});
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          trigger_estop("estop topic");
        }
      });
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      heartbeat_topic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        if (!last_heartbeat_) {
          RCLCPP_INFO(get_logger(), "GUI heartbeat received: dead-man armed");
        }
        last_heartbeat_ = SteadyClock::now();
      });

    estop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/estop", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        trigger_estop("estop service");
        res->success = true;
        res->message = "e-stop latched";
      });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_estop", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        reset_estop();
        res->success = true;
        res->message = "e-stop cleared (damping until new commands)";
      });

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_set_parameters(params);});

    lowcmd_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / lowcmd_rate_hz), [this]() {send_lowcmd();});
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publish_status();});

    RCLCPP_INFO(
      get_logger(), "Bridging %s -> %s, %s; commands %s -> %s (%.0f Hz). LowCmd output %s.",
      lowstate_topic.c_str(), imu_topic.c_str(), joint_states_topic.c_str(),
      command_topic.c_str(), lowcmd_topic.c_str(), lowcmd_rate_hz,
      enable_lowcmd_ ? "ENABLED" : "disabled (set enable_lowcmd:=true)");
  }

private:
  static constexpr size_t kNumMotorSlots = 35;

  std::string validate_gains_and_limits(
    const std::vector<double> & kp, const std::vector<double> & kd,
    const std::vector<double> & lower, const std::vector<double> & upper) const
  {
    const size_t n = joint_names_.size();
    if (kp.size() != n || kd.size() != n) {
      return "kp and kd must have len(joint_names) entries";
    }
    for (size_t i = 0; i < n; ++i) {
      if (kp[i] < 0.0 || kd[i] < 0.0) {
        return "kp and kd must be >= 0";
      }
    }
    if (lower.empty() && upper.empty()) {
      return "";
    }
    if (lower.size() != n || upper.size() != n) {
      return "position_lower/upper must be empty or len(joint_names)";
    }
    for (size_t i = 0; i < n; ++i) {
      if (lower[i] > upper[i]) {
        return "position_lower must be <= position_upper (" + joint_names_[i] + ")";
      }
    }
    return "";
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    static const std::set<std::string> kReadOnly = {
      "joint_names", "motor_indices", "mode_pr", "lowcmd_rate_hz", "lowstate_topic", "lowcmd_topic", "imu_topic",
      "joint_states_topic", "command_topic", "estop_topic", "heartbeat_topic", "imu_frame_id"};
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    // Validate everything on copies first, then apply atomically.
    auto kp = kp_;
    auto kd = kd_;
    auto lower = pos_lower_;
    auto upper = pos_upper_;
    double damping_kd = damping_kd_;
    double max_target_delta = max_target_delta_;
    double command_timeout = command_timeout_;
    double heartbeat_timeout = heartbeat_timeout_;
    bool enable_lowcmd = enable_lowcmd_;
    try {
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (kReadOnly.count(name)) {
          result.reason = name + " is read-only (restart the node to change it)";
          return result;
        } else if (name == "kp") {
          kp = p.as_double_array();
        } else if (name == "kd") {
          kd = p.as_double_array();
        } else if (name == "position_lower") {
          lower = p.as_double_array();
        } else if (name == "position_upper") {
          upper = p.as_double_array();
        } else if (name == "damping_kd") {
          damping_kd = p.as_double();
        } else if (name == "max_target_delta") {
          max_target_delta = p.as_double();
        } else if (name == "command_timeout_s") {
          command_timeout = p.as_double();
        } else if (name == "heartbeat_timeout_s") {
          heartbeat_timeout = p.as_double();
        } else if (name == "enable_lowcmd") {
          enable_lowcmd = p.as_bool();
        }
      }
    } catch (const rclcpp::ParameterTypeException & e) {
      result.reason = e.what();
      return result;
    }
    result.reason = validate_gains_and_limits(kp, kd, lower, upper);
    if (result.reason.empty() && (damping_kd < 0.0 || max_target_delta < 0.0)) {
      result.reason = "damping_kd and max_target_delta must be >= 0";
    }
    if (result.reason.empty() && command_timeout <= 0.0) {
      result.reason = "command_timeout_s must be > 0";
    }
    if (result.reason.empty() && heartbeat_timeout < 0.0) {
      result.reason = "heartbeat_timeout_s must be >= 0 (0 disables the dead-man)";
    }
    if (!result.reason.empty()) {
      return result;
    }

    kp_ = std::move(kp);
    kd_ = std::move(kd);
    pos_lower_ = std::move(lower);
    pos_upper_ = std::move(upper);
    damping_kd_ = damping_kd;
    max_target_delta_ = max_target_delta;
    command_timeout_ = command_timeout;
    heartbeat_timeout_ = heartbeat_timeout;
    if (enable_lowcmd != enable_lowcmd_) {
      enable_lowcmd_ = enable_lowcmd;
      RCLCPP_WARN(get_logger(), "LowCmd output %s", enable_lowcmd_ ? "ENABLED" : "disabled");
    }
    result.successful = true;
    return result;
  }

  void trigger_estop(const std::string & reason)
  {
    if (estop_) {
      return;
    }
    RCLCPP_ERROR(get_logger(), "EMERGENCY STOP (%s): damping until reset", reason.c_str());
    estop_reason_ = reason;
    estop_ = true;
    std_msgs::msg::Bool msg;
    msg.data = true;
    estop_pub_->publish(msg);
  }

  void reset_estop()
  {
    if (estop_) {
      RCLCPP_WARN(get_logger(), "E-stop reset (was: %s)", estop_reason_.c_str());
    }
    estop_ = false;
    estop_reason_.clear();
    // Old commands must not be resumed after a reset.
    last_command_.reset();
    // Re-arm the dead-man only once a fresh heartbeat arrives.
    last_heartbeat_.reset();
  }

  void on_lowstate(const unitree_hg::msg::LowState & msg)
  {
    const auto stamp = now();
    mode_machine_ = msg.mode_machine;

    const auto & imu = msg.imu_state;
    imu_msg_.header.stamp = stamp;
    imu_msg_.orientation.w = imu.quaternion[0];
    imu_msg_.orientation.x = imu.quaternion[1];
    imu_msg_.orientation.y = imu.quaternion[2];
    imu_msg_.orientation.z = imu.quaternion[3];
    imu_msg_.angular_velocity.x = imu.gyroscope[0];
    imu_msg_.angular_velocity.y = imu.gyroscope[1];
    imu_msg_.angular_velocity.z = imu.gyroscope[2];
    imu_msg_.linear_acceleration.x = imu.accelerometer[0];
    imu_msg_.linear_acceleration.y = imu.accelerometer[1];
    imu_msg_.linear_acceleration.z = imu.accelerometer[2];
    imu_pub_->publish(imu_msg_);

    joint_state_msg_.header.stamp = stamp;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto & m = msg.motor_state[motor_index_[i]];
      q_[i] = m.q;
      joint_state_msg_.position[i] = m.q;
      joint_state_msg_.velocity[i] = m.dq;
      joint_state_msg_.effort[i] = m.tau_est;
    }
    joint_state_pub_->publish(joint_state_msg_);
    last_lowstate_ = SteadyClock::now();
  }

  void on_command(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Command name/position mismatch");
      return;
    }
    // A command must cover every bridged joint; otherwise it is ignored.
    std::vector<double> target(joint_names_.size(), 0.0);
    std::vector<bool> seen(joint_names_.size(), false);
    for (size_t k = 0; k < msg.name.size(); ++k) {
      auto it = name_to_motor_.find(msg.name[k]);
      if (it != name_to_motor_.end()) {
        target[it->second] = msg.position[k];
        seen[it->second] = true;
      }
    }
    if (std::find(seen.begin(), seen.end(), false) != seen.end()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Command does not contain all bridged joints");
      return;
    }
    target_ = std::move(target);
    last_command_ = SteadyClock::now();
  }

  static double age(const std::optional<SteadyClock::time_point> & t)
  {
    if (!t) {
      return -1.0;
    }
    return std::chrono::duration<double>(SteadyClock::now() - *t).count();
  }

  bool fresh(const std::optional<SteadyClock::time_point> & t, double timeout) const
  {
    return t && age(t) < timeout;
  }

  void check_deadman()
  {
    if (estop_ || !enable_lowcmd_ || heartbeat_timeout_ <= 0.0 || !last_heartbeat_) {
      return;
    }
    if (!fresh(last_heartbeat_, heartbeat_timeout_)) {
      trigger_estop("GUI heartbeat lost");
    }
  }

  void send_lowcmd()
  {
    check_deadman();
    if (!enable_lowcmd_) {
      return;
    }
    if (!last_lowstate_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No lowstate received yet, not sending LowCmd");
      return;
    }
    const bool track = !estop_ && fresh(last_command_, command_timeout_) &&
      fresh(last_lowstate_, command_timeout_);
    if (track != tracking_) {
      tracking_ = track;
      RCLCPP_WARN(
        get_logger(), "%s", tracking_ ? "Tracking joint commands" : "Damping (no command / e-stop)");
    }

    lowcmd_.mode_pr = mode_pr_;
    lowcmd_.mode_machine = mode_machine_;
    // Slots without a bridged joint (absent motors) are disabled.
    for (auto & m : lowcmd_.motor_cmd) {
      m.mode = 0;
      m.q = 0.0f;
      m.dq = 0.0f;
      m.tau = 0.0f;
      m.kp = 0.0f;
      m.kd = 0.0f;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto & m = lowcmd_.motor_cmd[motor_index_[i]];
      m.mode = 1;
      if (tracking_) {
        double q = target_[i];
        if (max_target_delta_ > 0.0) {
          q = std::clamp(q, q_[i] - max_target_delta_, q_[i] + max_target_delta_);
        }
        if (!pos_lower_.empty()) {
          q = std::clamp(q, pos_lower_[i], pos_upper_[i]);
        }
        m.q = static_cast<float>(q);
        m.kp = static_cast<float>(kp_[i]);
        m.kd = static_cast<float>(kd_[i]);
      } else {
        m.q = static_cast<float>(q_[i]);
        m.kp = 0.0f;
        m.kd = static_cast<float>(damping_kd_);
      }
    }
    set_crc(lowcmd_);
    lowcmd_pub_->publish(lowcmd_);
  }

  void publish_status()
  {
    const char * state = "disabled";
    if (estop_) {
      state = "estop";
    } else if (enable_lowcmd_) {
      state = tracking_ ? "tracking" : "damping";
    }
    std::string reason;
    for (char c : estop_reason_) {
      if (c != '"' && c != '\\') {
        reason += c;
      }
    }
    char buf[512];
    std::snprintf(
      buf, sizeof(buf),
      "{\"state\":\"%s\",\"estop\":%s,\"estop_reason\":\"%s\",\"enable_lowcmd\":%s,"
      "\"deadman_armed\":%s,\"heartbeat_timeout_s\":%.3f,\"lowstate_age\":%.3f,"
      "\"command_age\":%.3f,\"heartbeat_age\":%.3f,\"mode_machine\":%d}",
      state, estop_ ? "true" : "false", reason.c_str(), enable_lowcmd_ ? "true" : "false",
      (heartbeat_timeout_ > 0.0 && last_heartbeat_) ? "true" : "false", heartbeat_timeout_,
      age(last_lowstate_), age(last_command_), age(last_heartbeat_), mode_machine_);
    std_msgs::msg::String msg;
    msg.data = buf;
    status_pub_->publish(msg);
  }

  // Parameters.
  std::vector<std::string> joint_names_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double damping_kd_{1.0};
  std::vector<double> pos_lower_;
  std::vector<double> pos_upper_;
  double max_target_delta_{0.0};
  uint8_t mode_pr_{0};
  bool enable_lowcmd_{false};
  double command_timeout_{0.1};
  double heartbeat_timeout_{0.5};
  std::string imu_frame_id_;

  // State.
  std::unordered_map<std::string, size_t> name_to_motor_;  // joint name -> joint index
  std::vector<size_t> motor_index_;  // joint index -> motor slot
  std::vector<double> q_;
  std::vector<double> target_;
  uint8_t mode_machine_{0};
  bool tracking_{false};
  bool estop_{false};
  std::string estop_reason_;
  std::optional<SteadyClock::time_point> last_lowstate_;
  std::optional<SteadyClock::time_point> last_command_;
  std::optional<SteadyClock::time_point> last_heartbeat_;

  // ROS interfaces.
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;
  unitree_hg::msg::LowCmd lowcmd_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr lowcmd_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace getup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<getup::G1LowLevelBridge>());
  rclcpp::shutdown();
  return 0;
}
