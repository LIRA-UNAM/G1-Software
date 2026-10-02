// Bridge between the Unitree G1 low-level DDS interface (unitree_hg) and
// generic ROS 2 messages.
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

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "getup/motor_crc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
    // Motor index i <-> joint_names[i].
    joint_names_ =
      declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    kp_ = declare_parameter<std::vector<double>>("kp", std::vector<double>{});
    kd_ = declare_parameter<std::vector<double>>("kd", std::vector<double>{});
    damping_kd_ = declare_parameter<double>("damping_kd", 1.0);
    pos_lower_ = declare_parameter<std::vector<double>>("position_lower", std::vector<double>{});
    pos_upper_ = declare_parameter<std::vector<double>>("position_upper", std::vector<double>{});
    max_target_delta_ = declare_parameter<double>("max_target_delta", 0.0);
    mode_pr_ = static_cast<uint8_t>(declare_parameter<int>("mode_pr", 0));
    enable_lowcmd_ = declare_parameter<bool>("enable_lowcmd", false);
    command_timeout_ = declare_parameter<double>("command_timeout_s", 0.1);
    const auto lowcmd_rate_hz = declare_parameter<double>("lowcmd_rate_hz", 500.0);
    const auto lowstate_topic = declare_parameter<std::string>("lowstate_topic", "/lowstate");
    const auto lowcmd_topic = declare_parameter<std::string>("lowcmd_topic", "/lowcmd");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const auto joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/getup/joint_command");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_in_pelvis");

    const size_t n = joint_names_.size();
    if (n == 0 || n > kNumMotorSlots) {
      throw std::invalid_argument("joint_names must have 1..35 entries");
    }
    if (kp_.size() != n || kd_.size() != n) {
      throw std::invalid_argument("kp and kd must have len(joint_names) entries");
    }
    has_limits_ = !pos_lower_.empty() || !pos_upper_.empty();
    if (has_limits_ && (pos_lower_.size() != n || pos_upper_.size() != n)) {
      throw std::invalid_argument("position_lower/upper must be empty or len(joint_names)");
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

    lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
      lowstate_topic, 10,
      [this](unitree_hg::msg::LowState::ConstSharedPtr msg) {on_lowstate(*msg);});
    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      command_topic, 10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_command(*msg);});

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "enable_lowcmd") {
            enable_lowcmd_ = p.as_bool();
            RCLCPP_WARN(get_logger(), "LowCmd output %s", enable_lowcmd_ ? "ENABLED" : "disabled");
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    lowcmd_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / lowcmd_rate_hz), [this]() {send_lowcmd();});

    RCLCPP_INFO(
      get_logger(), "Bridging %s -> %s, %s; commands %s -> %s (%.0f Hz). LowCmd output %s.",
      lowstate_topic.c_str(), imu_topic.c_str(), joint_states_topic.c_str(),
      command_topic.c_str(), lowcmd_topic.c_str(), lowcmd_rate_hz,
      enable_lowcmd_ ? "ENABLED" : "disabled (set enable_lowcmd:=true)");
  }

private:
  static constexpr size_t kNumMotorSlots = 35;

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
      const auto & m = msg.motor_state[i];
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

  bool fresh(const std::optional<SteadyClock::time_point> & t) const
  {
    return t && SteadyClock::now() - *t < std::chrono::duration<double>(command_timeout_);
  }

  void send_lowcmd()
  {
    if (!enable_lowcmd_) {
      return;
    }
    if (!last_lowstate_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No lowstate received yet, not sending LowCmd");
      return;
    }
    const bool track = fresh(last_command_) && fresh(last_lowstate_);
    if (track != tracking_) {
      tracking_ = track;
      RCLCPP_WARN(
        get_logger(), "%s", tracking_ ? "Tracking joint commands" : "No fresh command: damping");
    }

    lowcmd_.mode_pr = mode_pr_;
    lowcmd_.mode_machine = mode_machine_;
    for (size_t i = 0; i < kNumMotorSlots; ++i) {
      auto & m = lowcmd_.motor_cmd[i];
      m.dq = 0.0f;
      m.tau = 0.0f;
      if (i >= joint_names_.size()) {
        m.mode = 0;
        m.q = 0.0f;
        m.kp = 0.0f;
        m.kd = 0.0f;
        continue;
      }
      m.mode = 1;
      if (tracking_) {
        double q = target_[i];
        if (max_target_delta_ > 0.0) {
          q = std::clamp(q, q_[i] - max_target_delta_, q_[i] + max_target_delta_);
        }
        if (has_limits_) {
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

  // Parameters.
  std::vector<std::string> joint_names_;
  std::vector<double> kp_;
  std::vector<double> kd_;
  double damping_kd_{1.0};
  std::vector<double> pos_lower_;
  std::vector<double> pos_upper_;
  bool has_limits_{false};
  double max_target_delta_{0.0};
  uint8_t mode_pr_{0};
  bool enable_lowcmd_{false};
  double command_timeout_{0.1};
  std::string imu_frame_id_;

  // State.
  std::unordered_map<std::string, size_t> name_to_motor_;
  std::vector<double> q_;
  std::vector<double> target_;
  uint8_t mode_machine_{0};
  bool tracking_{false};
  std::optional<SteadyClock::time_point> last_lowstate_;
  std::optional<SteadyClock::time_point> last_command_;

  // ROS interfaces.
  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::JointState joint_state_msg_;
  unitree_hg::msg::LowCmd lowcmd_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr lowcmd_timer_;
};

}  // namespace getup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<getup::G1LowLevelBridge>());
  rclcpp::shutdown();
  return 0;
}
