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
//
// Hyperparameters can be changed at runtime through ROS parameters (see
// on_set_parameters); policy_path is only reloaded while the policy is idle.
// A true on /getup/estop stops the policy immediately.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "getup/observation.hpp"
#include "getup/onnx_policy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
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
    policy_path_ = declare_parameter<std::string>("policy_path", "");
    joint_names_ =
      declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    default_joint_pos_ =
      declare_parameter<std::vector<double>>("default_joint_pos", std::vector<double>{});
    observation_terms_ = declare_parameter<std::vector<std::string>>(
      "observation_terms", std::vector<std::string>{});
    observation_scales_ =
      declare_parameter<std::vector<double>>("observation_scales", std::vector<double>{});
    action_scale_ = declare_parameter<std::vector<double>>("action_scale", std::vector<double>{});
    action_clip_ = declare_parameter<double>("action_clip", 0.0);
    settle_steps_ = static_cast<int>(declare_parameter<int>("settle_steps", 0));
    const auto num_obs = declare_parameter<int>("num_obs", 0);
    policy_rate_hz_ = declare_parameter<double>("policy_rate_hz", 50.0);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 200.0);
    num_threads_ = static_cast<int>(declare_parameter<int>("num_threads", 1));
    // Runtime / IO.
    data_timeout_ = declare_parameter<double>("data_timeout_s", 0.1);
    auto_start_ = declare_parameter<bool>("auto_start", false);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const auto joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/getup/joint_command");
    const auto estop_topic = declare_parameter<std::string>("estop_topic", "/getup/estop");
    const bool publish_debug = declare_parameter<bool>("publish_debug", true);

    const size_t n = joint_names_.size();
    if (n == 0) {
      throw std::invalid_argument("joint_names must not be empty");
    }
    std::string error;
    action_scale_ = expand_action_scale(action_scale_, error);
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    if (policy_path_.empty()) {
      throw std::invalid_argument("policy_path is not set");
    }
    obs_builder_ = make_observation_builder(default_joint_pos_, observation_scales_, error);
    if (!obs_builder_) {
      throw std::invalid_argument(error);
    }
    if (num_obs > 0 && static_cast<size_t>(num_obs) != obs_builder_->size()) {
      throw std::invalid_argument(
              "num_obs=" + std::to_string(num_obs) + " but observation_terms give " +
              std::to_string(obs_builder_->size()));
    }
    policy_ = load_policy(policy_path_, error);
    if (!policy_) {
      throw std::invalid_argument(error);
    }

    state_.joint_pos.assign(n, 0.0);
    state_.joint_vel.assign(n, 0.0);
    action_.assign(n, 0.0f);
    obs_.assign(obs_builder_->size(), 0.0f);

    std::string terms_str;
    for (const auto & t : observation_terms_) {
      terms_str += t + " ";
    }
    RCLCPP_INFO(
      get_logger(), "Loaded policy %s (%s[%zu] -> %s[%zu]); obs terms: %s", policy_path_.c_str(),
      policy_->input_name().c_str(), policy_->num_obs(), policy_->output_name().c_str(),
      policy_->num_actions(), terms_str.c_str());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_state(*msg);});
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data) {
          stop("emergency stop");
        }
      });

    command_pub_ = create_publisher<sensor_msgs::msg::JointState>(command_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
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

    create_timers();
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publish_status();});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_set_parameters(params);});

    RCLCPP_INFO(
      get_logger(), "Policy %.0f Hz, command %.0f Hz, settle %d steps. %s",
      policy_rate_hz_, command_rate_hz_, settle_steps_,
      auto_start_ ? "Auto-start when data arrives." : "Call ~/start to run the policy.");
  }

private:
  enum class Mode { kIdle, kSettling, kRunning };

  std::vector<double> expand_action_scale(std::vector<double> scale, std::string & error) const
  {
    if (scale.size() == 1) {
      scale.assign(joint_names_.size(), scale[0]);
    }
    if (scale.size() != joint_names_.size()) {
      error = "action_scale must have 1 or len(joint_names) entries";
    }
    return scale;
  }

  std::unique_ptr<ObservationBuilder> make_observation_builder(
    const std::vector<double> & default_joint_pos, const std::vector<double> & scales,
    std::string & error) const
  {
    if (default_joint_pos.size() != joint_names_.size()) {
      error = "default_joint_pos must have len(joint_names) entries";
      return nullptr;
    }
    try {
      return std::make_unique<ObservationBuilder>(observation_terms_, default_joint_pos, scales);
    } catch (const std::exception & e) {
      error = e.what();
      return nullptr;
    }
  }

  // Loads a policy and checks it matches the observation layout and joints.
  std::unique_ptr<OnnxPolicy> load_policy(const std::string & path, std::string & error) const
  {
    std::unique_ptr<OnnxPolicy> policy;
    try {
      policy = std::make_unique<OnnxPolicy>(path, num_threads_);
    } catch (const std::exception & e) {
      error = "Cannot load policy " + path + ": " + e.what();
      return nullptr;
    }
    if (policy->num_obs() != obs_builder_->size()) {
      error = "Policy expects " + std::to_string(policy->num_obs()) +
        " observations, observation_terms give " + std::to_string(obs_builder_->size());
      return nullptr;
    }
    if (policy->num_actions() != joint_names_.size()) {
      error = "Policy outputs " + std::to_string(policy->num_actions()) + " actions but " +
        std::to_string(joint_names_.size()) + " joints are configured";
      return nullptr;
    }
    return policy;
  }

  void create_timers()
  {
    policy_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / policy_rate_hz_), [this]() {policy_step();});
    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / command_rate_hz_), [this]() {publish_command();});
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    static const std::set<std::string> kReadOnly = {
      "joint_names", "observation_terms", "num_obs", "num_threads", "imu_topic",
      "joint_states_topic", "command_topic", "estop_topic", "publish_debug"};
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    // Validate everything on copies first, then apply atomically.
    auto action_scale = action_scale_;
    auto default_joint_pos = default_joint_pos_;
    auto observation_scales = observation_scales_;
    double action_clip = action_clip_;
    int64_t settle_steps = settle_steps_;
    double data_timeout = data_timeout_;
    bool auto_start = auto_start_;
    double policy_rate = policy_rate_hz_;
    double command_rate = command_rate_hz_;
    std::optional<std::string> policy_path;
    bool rebuild_obs = false;
    try {
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (kReadOnly.count(name)) {
          result.reason = name + " is read-only (restart the node to change it)";
          return result;
        } else if (name == "action_scale") {
          action_scale = expand_action_scale(p.as_double_array(), result.reason);
        } else if (name == "default_joint_pos") {
          default_joint_pos = p.as_double_array();
          rebuild_obs = true;
        } else if (name == "observation_scales") {
          observation_scales = p.as_double_array();
          rebuild_obs = true;
        } else if (name == "action_clip") {
          action_clip = p.as_double();
        } else if (name == "settle_steps") {
          settle_steps = p.as_int();
        } else if (name == "data_timeout_s") {
          data_timeout = p.as_double();
        } else if (name == "auto_start") {
          auto_start = p.as_bool();
        } else if (name == "policy_rate_hz") {
          policy_rate = p.as_double();
        } else if (name == "command_rate_hz") {
          command_rate = p.as_double();
        } else if (name == "policy_path") {
          policy_path = p.as_string();
        }
        if (!result.reason.empty()) {
          return result;
        }
      }
    } catch (const rclcpp::ParameterTypeException & e) {
      result.reason = e.what();
      return result;
    }
    if (action_clip < 0.0 || settle_steps < 0 || data_timeout <= 0.0) {
      result.reason = "action_clip >= 0, settle_steps >= 0 and data_timeout_s > 0 required";
      return result;
    }
    if (policy_rate <= 0.0 || command_rate <= 0.0 || policy_rate > 1000.0 ||
      command_rate > 2000.0)
    {
      result.reason = "policy_rate_hz must be in (0, 1000] and command_rate_hz in (0, 2000]";
      return result;
    }
    std::unique_ptr<ObservationBuilder> obs_builder;
    if (rebuild_obs) {
      obs_builder = make_observation_builder(default_joint_pos, observation_scales, result.reason);
      if (!obs_builder) {
        return result;
      }
    }
    std::unique_ptr<OnnxPolicy> policy;
    if (policy_path && *policy_path != policy_path_) {
      if (mode_ != Mode::kIdle) {
        result.reason = "policy_path can only be changed while the policy is stopped";
        return result;
      }
      policy = load_policy(*policy_path, result.reason);
      if (!policy) {
        return result;
      }
    }

    // Apply.
    action_scale_ = std::move(action_scale);
    action_clip_ = action_clip;
    settle_steps_ = static_cast<int>(settle_steps);
    data_timeout_ = data_timeout;
    auto_start_ = auto_start;
    if (obs_builder) {
      default_joint_pos_ = std::move(default_joint_pos);
      observation_scales_ = std::move(observation_scales);
      obs_builder_ = std::move(obs_builder);
    }
    if (policy) {
      policy_ = std::move(policy);
      policy_path_ = *policy_path;
      RCLCPP_WARN(get_logger(), "Reloaded policy %s", policy_path_.c_str());
    }
    if (policy_rate != policy_rate_hz_ || command_rate != command_rate_hz_) {
      policy_rate_hz_ = policy_rate;
      command_rate_hz_ = command_rate;
      create_timers();
      RCLCPP_WARN(
        get_logger(), "Rates changed: policy %.1f Hz, command %.1f Hz", policy_rate_hz_,
        command_rate_hz_);
    }
    result.successful = true;
    return result;
  }

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

  static double age(const std::optional<SteadyClock::time_point> & t)
  {
    if (!t) {
      return -1.0;
    }
    return std::chrono::duration<double>(SteadyClock::now() - *t).count();
  }

  bool data_fresh() const
  {
    return last_imu_ && last_js_ && age(last_imu_) < data_timeout_ &&
           age(last_js_) < data_timeout_;
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
    // An e-stop must not be undone by auto_start.
    auto_start_ = false;
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

    const auto t0 = SteadyClock::now();
    obs_builder_->build(state_, action_, obs_);
    const auto & out = policy_->infer(obs_);
    inference_ms_ = std::chrono::duration<double, std::milli>(SteadyClock::now() - t0).count();
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

  void publish_status()
  {
    static const char * kModes[] = {"IDLE", "SETTLING", "RUNNING"};
    std::string path;
    for (char c : policy_path_) {
      if (c != '"' && c != '\\') {
        path += c;
      }
    }
    char buf[1024];
    std::snprintf(
      buf, sizeof(buf),
      "{\"mode\":\"%s\",\"step\":%d,\"settle_steps\":%d,\"data_fresh\":%s,\"imu_age\":%.3f,"
      "\"joint_states_age\":%.3f,\"inference_ms\":%.3f,\"auto_start\":%s,\"policy_path\":\"%s\"}",
      kModes[static_cast<int>(mode_)], step_, settle_steps_, data_fresh() ? "true" : "false",
      age(last_imu_), age(last_js_), inference_ms_, auto_start_ ? "true" : "false", path.c_str());
    std_msgs::msg::String msg;
    msg.data = buf;
    status_pub_->publish(msg);
  }

  // Parameters.
  std::string policy_path_;
  std::vector<std::string> joint_names_;
  std::vector<double> default_joint_pos_;
  std::vector<std::string> observation_terms_;
  std::vector<double> observation_scales_;
  std::vector<double> action_scale_;
  double action_clip_{0.0};
  int settle_steps_{0};
  double policy_rate_hz_{50.0};
  double command_rate_hz_{200.0};
  int num_threads_{1};
  double data_timeout_{0.1};
  bool auto_start_{false};

  // Policy.
  std::unique_ptr<ObservationBuilder> obs_builder_;
  std::unique_ptr<OnnxPolicy> policy_;
  std::vector<float> obs_;
  std::vector<float> action_;  // last raw policy output
  int step_{0};
  double inference_ms_{0.0};
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
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
