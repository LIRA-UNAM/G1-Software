// Runs an mjlab-trained velocity-tracking (walking) policy on the Unitree G1.
//
// Inputs:  sensor_msgs/Imu (pelvis IMU), sensor_msgs/JointState, a velocity
//          command (ROS parameters cmd_vx/cmd_vy/cmd_wz, set by the GUI, or
//          optionally geometry_msgs/Twist on cmd_vel_topic).
// Output:  sensor_msgs/JointState position targets for all joints, consumed by
//          getup's g1_lowlevel_bridge.
//
// The policy (task Pumas-Velocity-*-Unitree-G1) controls the 12 leg joints with
// an absolute position action: target = default_joint_pos + action_scale * a.
// Waist and arms are held at default_joint_pos, as in training.
//
// States:
//   IDLE     nothing is published (the bridge damps the joints)
//   HOMING   slow cosine move to home_pose, then HOLDING
//   HOLDING  the last target is held
//   RAMPING  policy running; the command blends from the start pose into the
//            policy targets over start_ramp_s
//   RUNNING  policy targets
// The policy can only be started while the IMU reports the robot upright
// (tilt <= max_start_tilt_deg). The tilt is computed continuously from the IMU
// and published in ~/status so the GUI can disable Start.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "getup/observation.hpp"
#include "getup/onnx_policy.hpp"
#include "ppowalk/motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ppowalk
{

using SteadyClock = std::chrono::steady_clock;

class PpowalkPolicyNode : public rclcpp::Node
{
public:
  PpowalkPolicyNode()
  : Node("ppowalk_policy_node")
  {
    policy_path_ = declare_parameter<std::string>("policy_path", "");
    joint_names_ =
      declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    action_joint_names_ = declare_parameter<std::vector<std::string>>(
      "action_joint_names", std::vector<std::string>{});
    default_joint_pos_ =
      declare_parameter<std::vector<double>>("default_joint_pos", std::vector<double>{});
    observation_terms_ = declare_parameter<std::vector<std::string>>(
      "observation_terms", std::vector<std::string>{});
    observation_scales_ =
      declare_parameter<std::vector<double>>("observation_scales", std::vector<double>{});
    action_scale_ = declare_parameter<std::vector<double>>("action_scale", std::vector<double>{});
    action_clip_ = declare_parameter<double>("action_clip", 0.0);
    const auto num_obs = declare_parameter<int>("num_obs", 0);
    policy_rate_hz_ = declare_parameter<double>("policy_rate_hz", 50.0);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 200.0);
    num_threads_ = static_cast<int>(declare_parameter<int>("num_threads", 1));
    data_timeout_ = declare_parameter<double>("data_timeout_s", 0.1);
    // Velocity command (live, from the GUI) and its limits (training ranges).
    cmd_ = {declare_parameter<double>("cmd_vx", 0.0), declare_parameter<double>("cmd_vy", 0.0),
      declare_parameter<double>("cmd_wz", 0.0)};
    cmd_min_ = {declare_parameter<double>("lin_vel_x_min", -1.0),
      declare_parameter<double>("lin_vel_y_min", -1.0),
      declare_parameter<double>("ang_vel_z_min", -0.5)};
    cmd_max_ = {declare_parameter<double>("lin_vel_x_max", 1.0),
      declare_parameter<double>("lin_vel_y_max", 1.0),
      declare_parameter<double>("ang_vel_z_max", 0.5)};
    use_cmd_vel_ = declare_parameter<bool>("use_cmd_vel", false);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);
    // Safety / motion.
    max_start_tilt_deg_ = declare_parameter<double>("max_start_tilt_deg", 30.0);
    start_ramp_s_ = declare_parameter<double>("start_ramp_s", 1.0);
    home_pose_ = declare_parameter<std::vector<double>>("home_pose", std::vector<double>{});
    home_max_vel_ = declare_parameter<double>("home_max_vel", 0.3);
    home_min_duration_ = declare_parameter<double>("home_min_duration_s", 2.0);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const auto joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/ppowalk/joint_command");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const bool publish_debug = declare_parameter<bool>("publish_debug", true);

    const size_t n = joint_names_.size();
    if (n == 0 || default_joint_pos_.size() != n) {
      throw std::invalid_argument("joint_names and default_joint_pos must be non-empty and match");
    }
    if (home_pose_.empty()) {
      home_pose_.assign(n, 0.0);
    }
    std::string error = check_home(home_pose_);
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    std::unordered_map<std::string, size_t> index;
    for (size_t i = 0; i < n; ++i) {
      index[joint_names_[i]] = i;
    }
    for (const auto & name : action_joint_names_) {
      auto it = index.find(name);
      if (it == index.end()) {
        throw std::invalid_argument("action joint '" + name + "' is not in joint_names");
      }
      action_index_.push_back(it->second);
    }
    if (action_index_.empty()) {
      throw std::invalid_argument("action_joint_names must not be empty");
    }
    action_scale_ = expand_action_scale(action_scale_, error);
    if (!error.empty()) {
      throw std::invalid_argument(error);
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
    if (!policy_path_.empty()) {
      policy_ = load_policy(policy_path_, error);
      if (!policy_) {
        // Keep running (Home must work); Start is refused until a valid model
        // is loaded through the policy_path parameter.
        RCLCPP_ERROR(get_logger(), "%s", error.c_str());
        policy_path_.clear();
      }
    }
    if (!policy_) {
      RCLCPP_WARN(get_logger(), "No policy loaded: Home works, Start is refused until a model is loaded");
    }

    state_.joint_pos.assign(n, 0.0);
    state_.joint_vel.assign(n, 0.0);
    action_.assign(action_index_.size(), 0.0f);
    obs_.assign(obs_builder_->size(), 0.0f);
    policy_target_ = default_joint_pos_;
    command_msg_.name = joint_names_;
    command_msg_.position.assign(n, 0.0);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint_state(*msg);});
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        cmd_vel_ = {msg->linear.x, msg->linear.y, msg->angular.z};
        last_cmd_vel_ = SteadyClock::now();
      });

    command_pub_ = create_publisher<sensor_msgs::msg::JointState>(command_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    if (publish_debug) {
      obs_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/observation", 10);
      action_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/action", 10);
    }

    auto service = [this](const std::string & name, auto handler) {
        return create_service<std_srvs::srv::Trigger>(
          "~/" + name, [this, handler](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            res->success = (this->*handler)(res->message);
          });
      };
    start_srv_ = service("start", &PpowalkPolicyNode::start);
    stop_srv_ = service("stop", &PpowalkPolicyNode::stop);
    release_srv_ = service("release", &PpowalkPolicyNode::release);
    home_srv_ = service("home", &PpowalkPolicyNode::home);

    create_timers();
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publish_status();});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_set_parameters(params);});

    RCLCPP_INFO(
      get_logger(), "ppowalk ready: %zu joints, %zu actions, %zu obs, policy %.0f Hz, start "
      "allowed below %.0f deg tilt. Model: %s", n, action_index_.size(), obs_builder_->size(),
      policy_rate_hz_, max_start_tilt_deg_, policy_path_.empty() ? "(none)" : policy_path_.c_str());
  }

private:
  enum class Mode { kIdle, kHoming, kHolding, kRamping, kRunning };
  static const char * mode_name(Mode m)
  {
    static const char * names[] = {"IDLE", "HOMING", "HOLDING", "RAMPING", "RUNNING"};
    return names[static_cast<int>(m)];
  }

  // ------------------------------------------------------------ validation

  std::string check_home(const std::vector<double> & home) const
  {
    return home.size() == joint_names_.size() ? "" : "home_pose must have len(joint_names) entries";
  }

  std::vector<double> expand_action_scale(std::vector<double> scale, std::string & error) const
  {
    if (scale.size() == 1) {
      scale.assign(action_index_.size(), scale[0]);
    }
    if (scale.size() != action_index_.size()) {
      error = "action_scale must have 1 or len(action_joint_names) entries";
    }
    return scale;
  }

  std::unique_ptr<getup::ObservationBuilder> make_observation_builder(
    const std::vector<double> & default_joint_pos, const std::vector<double> & scales,
    std::string & error) const
  {
    if (default_joint_pos.size() != joint_names_.size()) {
      error = "default_joint_pos must have len(joint_names) entries";
      return nullptr;
    }
    try {
      return std::make_unique<getup::ObservationBuilder>(
        observation_terms_, default_joint_pos, scales, action_index_.size());
    } catch (const std::exception & e) {
      error = e.what();
      return nullptr;
    }
  }

  std::unique_ptr<getup::OnnxPolicy> load_policy(const std::string & path, std::string & error) const
  {
    std::unique_ptr<getup::OnnxPolicy> policy;
    try {
      policy = std::make_unique<getup::OnnxPolicy>(path, num_threads_);
    } catch (const std::exception & e) {
      error = "Cannot load policy " + path + ": " + e.what();
      return nullptr;
    }
    if (policy->num_obs() != obs_builder_->size() ||
      policy->num_actions() != action_index_.size())
    {
      error = "Policy is " + std::to_string(policy->num_obs()) + " -> " +
        std::to_string(policy->num_actions()) + ", expected " +
        std::to_string(obs_builder_->size()) + " -> " + std::to_string(action_index_.size());
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
      "joint_names", "action_joint_names", "observation_terms", "num_obs", "num_threads",
      "imu_topic", "joint_states_topic", "command_topic", "cmd_vel_topic", "publish_debug"};
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    auto action_scale = action_scale_;
    auto default_joint_pos = default_joint_pos_;
    auto observation_scales = observation_scales_;
    auto home_pose = home_pose_;
    auto cmd = cmd_;
    auto cmd_min = cmd_min_;
    auto cmd_max = cmd_max_;
    double action_clip = action_clip_, data_timeout = data_timeout_;
    double policy_rate = policy_rate_hz_, command_rate = command_rate_hz_;
    double max_tilt = max_start_tilt_deg_, ramp = start_ramp_s_;
    double home_vel = home_max_vel_, home_min = home_min_duration_;
    double cmd_vel_timeout = cmd_vel_timeout_;
    bool use_cmd_vel = use_cmd_vel_;
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
        } else if (name == "home_pose") {
          home_pose = p.as_double_array();
          result.reason = check_home(home_pose);
        } else if (name == "action_clip") {
          action_clip = p.as_double();
        } else if (name == "data_timeout_s") {
          data_timeout = p.as_double();
        } else if (name == "policy_rate_hz") {
          policy_rate = p.as_double();
        } else if (name == "command_rate_hz") {
          command_rate = p.as_double();
        } else if (name == "max_start_tilt_deg") {
          max_tilt = p.as_double();
        } else if (name == "start_ramp_s") {
          ramp = p.as_double();
        } else if (name == "home_max_vel") {
          home_vel = p.as_double();
        } else if (name == "home_min_duration_s") {
          home_min = p.as_double();
        } else if (name == "cmd_vx") {
          cmd[0] = p.as_double();
        } else if (name == "cmd_vy") {
          cmd[1] = p.as_double();
        } else if (name == "cmd_wz") {
          cmd[2] = p.as_double();
        } else if (name == "lin_vel_x_min") {
          cmd_min[0] = p.as_double();
        } else if (name == "lin_vel_y_min") {
          cmd_min[1] = p.as_double();
        } else if (name == "ang_vel_z_min") {
          cmd_min[2] = p.as_double();
        } else if (name == "lin_vel_x_max") {
          cmd_max[0] = p.as_double();
        } else if (name == "lin_vel_y_max") {
          cmd_max[1] = p.as_double();
        } else if (name == "ang_vel_z_max") {
          cmd_max[2] = p.as_double();
        } else if (name == "use_cmd_vel") {
          use_cmd_vel = p.as_bool();
        } else if (name == "cmd_vel_timeout_s") {
          cmd_vel_timeout = p.as_double();
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
    if (action_clip < 0.0 || data_timeout <= 0.0 || ramp < 0.0 || home_min < 0.0 ||
      home_vel <= 0.0 || cmd_vel_timeout <= 0.0)
    {
      result.reason = "action_clip, start_ramp_s, home_min_duration_s >= 0; data_timeout_s, "
        "home_max_vel, cmd_vel_timeout_s > 0";
      return result;
    }
    if (max_tilt <= 0.0 || max_tilt > 90.0) {
      result.reason = "max_start_tilt_deg must be in (0, 90]";
      return result;
    }
    if (policy_rate <= 0.0 || command_rate <= 0.0 || policy_rate > 1000.0 ||
      command_rate > 2000.0)
    {
      result.reason = "policy_rate_hz must be in (0, 1000] and command_rate_hz in (0, 2000]";
      return result;
    }
    for (int k = 0; k < 3; ++k) {
      if (cmd_min[k] > 0.0 || cmd_max[k] < 0.0) {
        result.reason = "velocity limits must include 0 (min <= 0 <= max)";
        return result;
      }
    }
    std::unique_ptr<getup::ObservationBuilder> obs_builder;
    if (rebuild_obs) {
      obs_builder = make_observation_builder(default_joint_pos, observation_scales, result.reason);
      if (!obs_builder) {
        return result;
      }
    }
    std::unique_ptr<getup::OnnxPolicy> policy;
    if (policy_path && *policy_path != policy_path_) {
      if (mode_ == Mode::kRamping || mode_ == Mode::kRunning) {
        result.reason = "policy_path can only be changed while the policy is stopped";
        return result;
      }
      policy = load_policy(*policy_path, result.reason);
      if (!policy) {
        return result;
      }
    }

    action_scale_ = std::move(action_scale);
    home_pose_ = std::move(home_pose);
    cmd_ = cmd;
    cmd_min_ = cmd_min;
    cmd_max_ = cmd_max;
    action_clip_ = action_clip;
    data_timeout_ = data_timeout;
    max_start_tilt_deg_ = max_tilt;
    start_ramp_s_ = ramp;
    home_max_vel_ = home_vel;
    home_min_duration_ = home_min;
    use_cmd_vel_ = use_cmd_vel;
    cmd_vel_timeout_ = cmd_vel_timeout;
    if (obs_builder) {
      default_joint_pos_ = std::move(default_joint_pos);
      observation_scales_ = std::move(observation_scales);
      obs_builder_ = std::move(obs_builder);
    }
    if (policy) {
      policy_ = std::move(policy);
      policy_path_ = *policy_path;
      RCLCPP_WARN(get_logger(), "Loaded policy %s", policy_path_.c_str());
    }
    if (policy_rate != policy_rate_hz_ || command_rate != command_rate_hz_) {
      policy_rate_hz_ = policy_rate;
      command_rate_hz_ = command_rate;
      create_timers();
    }
    result.successful = true;
    return result;
  }

  // ---------------------------------------------------------------- inputs

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    state_.base_ang_vel = {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
    state_.base_quat = {msg.orientation.w, msg.orientation.x, msg.orientation.y,
      msg.orientation.z};
    const auto g = getup::projected_gravity(
      msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    tilt_deg_ = tilt_deg(g[0], g[1], g[2]);
    last_imu_ = SteadyClock::now();
  }

  void on_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size()) {
      return;
    }
    if (msg.name != last_js_names_) {
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
    return t ? std::chrono::duration<double>(SteadyClock::now() - *t).count() : -1.0;
  }

  bool data_fresh() const
  {
    return last_imu_ && last_js_ && age(last_imu_) < data_timeout_ &&
           age(last_js_) < data_timeout_;
  }

  bool upright() const
  {
    return last_imu_ && age(last_imu_) < data_timeout_ && tilt_deg_ <= max_start_tilt_deg_;
  }

  // Velocity command in use: cmd_vel (if enabled and fresh) else the GUI
  // parameters, clamped to the configured limits.
  std::array<double, 3> velocity_command() const
  {
    std::array<double, 3> c = cmd_;
    if (use_cmd_vel_) {
      c = (last_cmd_vel_ && age(last_cmd_vel_) < cmd_vel_timeout_) ?
        cmd_vel_ : std::array<double, 3>{0.0, 0.0, 0.0};
    }
    for (int k = 0; k < 3; ++k) {
      c[k] = std::clamp(c[k], cmd_min_[k], cmd_max_[k]);
    }
    return c;
  }

  // -------------------------------------------------------------- services

  std::vector<double> current_pose() const
  {
    return target_ ? *target_ : state_.joint_pos;
  }

  bool home(std::string & message)
  {
    if (!data_fresh()) {
      message = "no fresh IMU/JointState data";
      return false;
    }
    move_from_ = current_pose();
    move_duration_ = safe_move_duration(move_from_, home_pose_, home_max_vel_, home_min_duration_);
    move_start_ = SteadyClock::now();
    if (mode_ == Mode::kRamping || mode_ == Mode::kRunning) {
      RCLCPP_WARN(get_logger(), "Policy stopped: moving home");
    }
    mode_ = Mode::kHoming;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "moving home (%.1f s)", move_duration_);
    message = buf;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  bool start(std::string & message)
  {
    if (!policy_) {
      message = "no policy loaded (set policy_path)";
    } else if (mode_ == Mode::kRamping || mode_ == Mode::kRunning) {
      message = "already running";
    } else if (mode_ == Mode::kHoming) {
      message = "wait until homing is finished";
    } else if (!data_fresh()) {
      message = "no fresh IMU/JointState data";
    } else if (!upright()) {
      char buf[96];
      std::snprintf(
        buf, sizeof(buf), "robot not upright: tilt %.1f deg > %.1f deg", tilt_deg_,
        max_start_tilt_deg_);
      message = buf;
    } else {
      std::fill(action_.begin(), action_.end(), 0.0f);
      policy_target_ = default_joint_pos_;
      move_from_ = current_pose();
      move_start_ = SteadyClock::now();
      mode_ = start_ramp_s_ > 0.0 ? Mode::kRamping : Mode::kRunning;
      step_ = 0;
      message = "started";
      RCLCPP_INFO(get_logger(), "Policy started (tilt %.1f deg)", tilt_deg_);
      return true;
    }
    RCLCPP_WARN(get_logger(), "Start refused: %s", message.c_str());
    return false;
  }

  // Stop = freeze: keep commanding the last target (the robot stays stiff).
  bool stop(std::string & message)
  {
    if (mode_ == Mode::kIdle) {
      message = "nothing to stop";
      return false;
    }
    if (target_) {
      mode_ = Mode::kHolding;
      message = "holding the current pose";
    } else {
      mode_ = Mode::kIdle;
      message = "stopped";
    }
    RCLCPP_WARN(get_logger(), "Stop: %s", message.c_str());
    return true;
  }

  // Release = stop publishing: the bridge damps the joints (the robot goes limp).
  bool release(std::string & message)
  {
    mode_ = Mode::kIdle;
    target_.reset();
    message = "released (bridge damping)";
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return true;
  }

  // ------------------------------------------------------------- execution

  void policy_step()
  {
    if (mode_ != Mode::kRamping && mode_ != Mode::kRunning) {
      return;
    }
    if (!data_fresh()) {
      // On-robot sensor loss: freeze instead of walking blind.
      RCLCPP_ERROR(get_logger(), "Sensor data timed out: holding the current pose");
      mode_ = target_ ? Mode::kHolding : Mode::kIdle;
      return;
    }
    state_.command = velocity_command();
    const auto t0 = SteadyClock::now();
    obs_builder_->build(state_, action_, obs_);
    const auto & out = policy_->infer(obs_);
    inference_ms_ = std::chrono::duration<double, std::milli>(SteadyClock::now() - t0).count();
    auto target = default_joint_pos_;
    for (size_t k = 0; k < action_.size(); ++k) {
      float a = out[k];
      if (action_clip_ > 0.0) {
        a = std::clamp(a, static_cast<float>(-action_clip_), static_cast<float>(action_clip_));
      }
      action_[k] = a;
      target[action_index_[k]] += action_scale_[k] * a;
    }
    policy_target_ = std::move(target);
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
    const auto now = SteadyClock::now();
    const double t = std::chrono::duration<double>(now - move_start_).count();
    switch (mode_) {
      case Mode::kIdle:
        return;
      case Mode::kHoming: {
          const double s = t / std::max(move_duration_, 1e-6);
          target_ = blend(move_from_, home_pose_, smooth_alpha(s));
          if (s >= 1.0) {
            mode_ = Mode::kHolding;
            RCLCPP_INFO(get_logger(), "Home reached: holding");
          }
          break;
        }
      case Mode::kHolding:
        break;
      case Mode::kRamping: {
          const double s = t / std::max(start_ramp_s_, 1e-6);
          target_ = blend(move_from_, policy_target_, smooth_alpha(s));
          if (s >= 1.0) {
            mode_ = Mode::kRunning;
          }
          break;
        }
      case Mode::kRunning:
        target_ = policy_target_;
        break;
    }
    if (!target_) {
      return;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      command_msg_.position[i] = (*target_)[i];
    }
    command_msg_.header.stamp = this->now();
    command_pub_->publish(command_msg_);
  }

  void publish_status()
  {
    const auto c = velocity_command();
    std::string path;
    for (char ch : policy_path_) {
      if (ch != '"' && ch != '\\') {
        path += ch;
      }
    }
    const bool cmd_vel_live = use_cmd_vel_ && last_cmd_vel_ && age(last_cmd_vel_) < cmd_vel_timeout_;
    char buf[1024];
    std::snprintf(
      buf, sizeof(buf),
      "{\"mode\":\"%s\",\"step\":%d,\"upright\":%s,\"tilt_deg\":%.2f,\"max_start_tilt_deg\":%.1f,"
      "\"data_fresh\":%s,\"imu_age\":%.3f,\"joint_states_age\":%.3f,\"inference_ms\":%.3f,"
      "\"policy_loaded\":%s,\"policy_path\":\"%s\",\"command\":[%.3f,%.3f,%.3f],"
      "\"command_source\":\"%s\"}",
      mode_name(mode_), step_, upright() ? "true" : "false", last_imu_ ? tilt_deg_ : -1.0,
      max_start_tilt_deg_, data_fresh() ? "true" : "false", age(last_imu_), age(last_js_),
      inference_ms_, policy_ ? "true" : "false", path.c_str(), c[0], c[1], c[2],
      use_cmd_vel_ ? (cmd_vel_live ? "cmd_vel" : "cmd_vel (stale: 0)") : "gui");
    std_msgs::msg::String msg;
    msg.data = buf;
    status_pub_->publish(msg);
  }

  // Parameters.
  std::string policy_path_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> action_joint_names_;
  std::vector<size_t> action_index_;
  std::vector<double> default_joint_pos_;
  std::vector<std::string> observation_terms_;
  std::vector<double> observation_scales_;
  std::vector<double> action_scale_;
  double action_clip_{0.0};
  double policy_rate_hz_{50.0};
  double command_rate_hz_{200.0};
  int num_threads_{1};
  double data_timeout_{0.1};
  std::array<double, 3> cmd_{0.0, 0.0, 0.0};
  std::array<double, 3> cmd_min_{-1.0, -1.0, -0.5};
  std::array<double, 3> cmd_max_{1.0, 1.0, 0.5};
  bool use_cmd_vel_{false};
  double cmd_vel_timeout_{0.5};
  double max_start_tilt_deg_{30.0};
  double start_ramp_s_{1.0};
  std::vector<double> home_pose_;
  double home_max_vel_{0.3};
  double home_min_duration_{2.0};

  // State.
  std::unique_ptr<getup::ObservationBuilder> obs_builder_;
  std::unique_ptr<getup::OnnxPolicy> policy_;
  std::vector<float> obs_;
  std::vector<float> action_;
  std::vector<double> policy_target_;
  std::optional<std::vector<double>> target_;  // last published target
  std::vector<double> move_from_;
  SteadyClock::time_point move_start_{};
  double move_duration_{0.0};
  int step_{0};
  double inference_ms_{0.0};
  double tilt_deg_{180.0};
  Mode mode_{Mode::kIdle};
  getup::RobotState state_;
  std::vector<std::string> last_js_names_;
  std::vector<size_t> js_index_;
  std::optional<SteadyClock::time_point> last_imu_;
  std::optional<SteadyClock::time_point> last_js_;
  std::array<double, 3> cmd_vel_{0.0, 0.0, 0.0};
  std::optional<SteadyClock::time_point> last_cmd_vel_;

  // ROS interfaces.
  sensor_msgs::msg::JointState command_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace ppowalk

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ppowalk::PpowalkPolicyNode>());
  rclcpp::shutdown();
  return 0;
}
