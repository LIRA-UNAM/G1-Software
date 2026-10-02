// Runs an mjlab-trained getup policy (exported to ONNX) on the Unitree G1.
//
// Inputs:  sensor_msgs/Imu (pelvis IMU) and sensor_msgs/JointState.
// Output:  sensor_msgs/JointState with joint position targets (policy order).
//
// The action is a *relative* joint position target, as in mjlab's
// SettleRelativeJointPositionAction:
//   target = q_current + action * action_scale
// mjlab re-evaluates it on every physics substep (200 Hz) with the latest q
// while the action itself is held for one policy step (50 Hz). This node
// mirrors that with two timers. During the first `settle_steps` policy steps
// after start the policy runs (its output feeds the last-action observation)
// but the command is just the current position.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "getup/observation.hpp"
#include "getup/onnx_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace getup
{

using SteadyClock = std::chrono::steady_clock;

class GetupPolicyNode : public rclcpp::Node
{
public:
  GetupPolicyNode()
  : Node("getup_policy_node")
  {
    // Policy / observation hyperparameters (see config/g1_getup_policy.yaml).
    const auto policy_path = declare_parameter<std::string>("policy_path", "");
    joint_names_ =
      declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    const auto default_joint_pos =
      declare_parameter<std::vector<double>>("default_joint_pos", std::vector<double>{});
    const auto observation_terms =
      declare_parameter<std::vector<std::string>>("observation_terms", std::vector<std::string>{});
    const auto observation_scales =
      declare_parameter<std::vector<double>>("observation_scales", std::vector<double>{});
    action_scale_ = declare_parameter<std::vector<double>>("action_scale", std::vector<double>{});
    action_clip_ = declare_parameter<double>("action_clip", 0.0);
    settle_steps_ = static_cast<int>(declare_parameter<int>("settle_steps", 0));
    const auto num_obs = declare_parameter<int>("num_obs", 0);
    const auto policy_rate_hz = declare_parameter<double>("policy_rate_hz", 50.0);
    const auto command_rate_hz = declare_parameter<double>("command_rate_hz", 200.0);
    const auto num_threads = declare_parameter<int>("num_threads", 1);
    // Runtime / IO.
    data_timeout_ = declare_parameter<double>("data_timeout_s", 0.1);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const auto joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/getup/joint_command");
    const bool publish_debug = declare_parameter<bool>("publish_debug", true);

    const size_t n = joint_names_.size();
    if (n == 0 || default_joint_pos.size() != n) {
      throw std::invalid_argument("joint_names and default_joint_pos must be non-empty and match");
    }
    if (action_scale_.size() == 1) {
      action_scale_.assign(n, action_scale_[0]);
    }
    if (action_scale_.size() != n) {
      throw std::invalid_argument("action_scale must have 1 or len(joint_names) entries");
    }
    if (policy_path.empty()) {
      throw std::invalid_argument("policy_path is not set");
    }

    obs_builder_ = std::make_unique<ObservationBuilder>(
      observation_terms, default_joint_pos, observation_scales);
    policy_ = std::make_unique<OnnxPolicy>(policy_path, static_cast<int>(num_threads));

    if (num_obs > 0 && static_cast<size_t>(num_obs) != obs_builder_->size()) {
      throw std::invalid_argument(
              "num_obs=" + std::to_string(num_obs) + " but observation_terms give " +
              std::to_string(obs_builder_->size()));
    }
    if (policy_->num_obs() != obs_builder_->size()) {
      throw std::invalid_argument(
              "Policy expects " + std::to_string(policy_->num_obs()) +
              " observations, observation_terms give " + std::to_string(obs_builder_->size()));
    }
    if (policy_->num_actions() != n) {
      throw std::invalid_argument(
              "Policy outputs " + std::to_string(policy_->num_actions()) + " actions but " +
              std::to_string(n) + " joints are configured");
    }

    state_.joint_pos.assign(n, 0.0);
    state_.joint_vel.assign(n, 0.0);
    action_.assign(n, 0.0f);
    obs_.assign(obs_builder_->size(), 0.0f);

    std::string terms_str;
    for (const auto & t : observation_terms) {
      terms_str += t + " ";
    }
    RCLCPP_INFO(
      get_logger(), "Loaded policy %s (%s[%zu] -> %s[%zu]); obs terms: %s", policy_path.c_str(),
      policy_->input_name().c_str(), policy_->num_obs(), policy_->output_name().c_str(),
      policy_->num_actions(), terms_str.c_str());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_state(*msg);});

    command_pub_ = create_publisher<sensor_msgs::msg::JointState>(command_topic, 10);
    if (publish_debug) {
      obs_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/observation", 10);
      action_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/action", 10);
    }

    command_msg_.name = joint_names_;
    command_msg_.position.assign(n, 0.0);

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = start(res->message);
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        stop("stop service called");
        res->success = true;
        res->message = "stopped";
      });

    policy_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / policy_rate_hz), [this]() {policy_step();});
    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / command_rate_hz), [this]() {publish_command();});

    RCLCPP_INFO(
      get_logger(), "Policy %.0f Hz, command %.0f Hz, settle %d steps. %s",
      policy_rate_hz, command_rate_hz, settle_steps_,
      auto_start_ ? "Auto-start when data arrives." : "Call ~/start to run the policy.");
  }

private:
  enum class Mode { kIdle, kSettling, kRunning };

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    state_.base_ang_vel = {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
    state_.base_quat = {msg.orientation.w, msg.orientation.x, msg.orientation.y,
      msg.orientation.z};
    last_imu_ = SteadyClock::now();
  }

  void on_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {
      return;
    }
    if (msg.name != last_js_names_) {
      // Map policy joint i -> index in the incoming message (by name).
      std::unordered_map<std::string, size_t> index;
      for (size_t k = 0; k < msg.name.size(); ++k) {
        index[msg.name[k]] = k;
      }
      std::vector<size_t> js_index(joint_names_.size(), 0);
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = index.find(joint_names_[i]);
        if (it == index.end()) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "JointState is missing joint '%s'",
            joint_names_[i].c_str());
          return;
        }
        js_index[i] = it->second;
      }
      js_index_ = std::move(js_index);
      last_js_names_ = msg.name;
    }
    const bool has_vel = msg.velocity.size() == msg.name.size();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      state_.joint_pos[i] = msg.position[js_index_[i]];
      state_.joint_vel[i] = has_vel ? msg.velocity[js_index_[i]] : 0.0;
    }
    last_js_ = SteadyClock::now();
  }

  bool data_fresh() const
  {
    if (!last_imu_ || !last_js_) {
      return false;
    }
    const auto now = SteadyClock::now();
    const auto timeout = std::chrono::duration<double>(data_timeout_);
    return now - *last_imu_ < timeout && now - *last_js_ < timeout;
  }

  bool start(std::string & message)
  {
    if (!data_fresh()) {
      message = "no fresh IMU/JointState data";
      RCLCPP_WARN(get_logger(), "Cannot start: %s", message.c_str());
      return false;
    }
    std::fill(action_.begin(), action_.end(), 0.0f);
    step_ = 0;
    mode_ = settle_steps_ > 0 ? Mode::kSettling : Mode::kRunning;
    message = "started";
    RCLCPP_INFO(get_logger(), "Policy started (settling for %d steps)", settle_steps_);
    return true;
  }

  void stop(const std::string & reason)
  {
    if (mode_ != Mode::kIdle) {
      RCLCPP_WARN(get_logger(), "Policy stopped: %s", reason.c_str());
    }
    mode_ = Mode::kIdle;
  }

  void policy_step()
  {
    if (mode_ == Mode::kIdle) {
      if (auto_start_ && data_fresh()) {
        std::string msg;
        auto_start_ = !start(msg);
      }
      return;
    }
    if (!data_fresh()) {
      stop("sensor data timed out");
      return;
    }

    obs_builder_->build(state_, action_, obs_);
    const auto & out = policy_->infer(obs_);
    for (size_t i = 0; i < action_.size(); ++i) {
      float a = out[i];
      if (action_clip_ > 0.0) {
        a = std::clamp(a, static_cast<float>(-action_clip_), static_cast<float>(action_clip_));
      }
      action_[i] = a;
    }
    // step_ is the index of the action just computed, i.e. mjlab's
    // episode_length_buf while that action is applied.
    if (mode_ == Mode::kSettling && step_ >= settle_steps_) {
      mode_ = Mode::kRunning;
      RCLCPP_INFO(get_logger(), "Settle phase done, applying policy actions");
    }
    ++step_;

    if (obs_pub_) {
      std_msgs::msg::Float32MultiArray m;
      m.data = obs_;
      obs_pub_->publish(m);
      m.data = action_;
      action_pub_->publish(m);
    }
  }

  void publish_command()
  {
    if (mode_ == Mode::kIdle) {
      return;
    }
    const bool apply_action = mode_ == Mode::kRunning;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      command_msg_.position[i] =
        state_.joint_pos[i] + (apply_action ? action_scale_[i] * action_[i] : 0.0);
    }
    command_msg_.header.stamp = now();
    command_pub_->publish(command_msg_);
  }

  // Parameters.
  std::vector<std::string> joint_names_;
  std::vector<double> action_scale_;
  double action_clip_{0.0};
  int settle_steps_{0};
  double data_timeout_{0.1};
  bool auto_start_{false};

  // Policy.
  std::unique_ptr<ObservationBuilder> obs_builder_;
  std::unique_ptr<OnnxPolicy> policy_;
  std::vector<float> obs_;
  std::vector<float> action_;  // last raw policy output
  int step_{0};
  Mode mode_{Mode::kIdle};

  // Robot state.
  RobotState state_;
  std::vector<std::string> last_js_names_;
  std::vector<size_t> js_index_;
  std::optional<SteadyClock::time_point> last_imu_;
  std::optional<SteadyClock::time_point> last_js_;

  // ROS interfaces.
  sensor_msgs::msg::JointState command_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace getup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Single-threaded executor: callbacks never run concurrently, so the shared
  // state needs no locking.
  rclcpp::spin(std::make_shared<getup::GetupPolicyNode>());
  rclcpp::shutdown();
  return 0;
}
