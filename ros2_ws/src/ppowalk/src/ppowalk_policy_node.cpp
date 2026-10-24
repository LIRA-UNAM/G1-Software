// Runs an mjlab-trained velocity-tracking (walking) policy on the Unitree G1.
//
// Inputs:  sensor_msgs/Imu (pelvis IMU), sensor_msgs/JointState, a velocity
//          command from one source: the Unitree wireless remote (use_remote,
//          sensor_msgs/Joy republished by the bridge), geometry_msgs/Twist on
//          cmd_vel_topic (use_cmd_vel) or the ROS parameters cmd_vx/cmd_vy/
//          cmd_wz set by the GUI (both off).
// Output:  sensor_msgs/JointState position targets for all joints, consumed by
//          getup's g1_lowlevel_bridge.
//
// The policy (task Pumas-Velocity-*-Unitree-G1) controls the 12 leg joints with
// an absolute position action: target = default_joint_pos + action_scale * a.
// Only command_joints are published (default: legs + waist); the bridge damps
// the rest. always_commanded_joints (default: the waist, held at 0 as in
// training) can never be dropped from command_joints. Commanded joints the policy does not drive (waist, arms) are held at
// held_joint_targets, which is 0 in the training environment (not the
// default pose).
//
// Pinned observations: the joint_pos / joint_vel observation of a joint the
// policy cannot rely on is reported as if it sat at its hold target, at rest,
// as in training. That covers every non-commanded joint (limp under damping),
// every motor detected offline (it reports exactly q = dq = tau = 0; latched
// for the whole run so a flapping motor cannot leak through) and the manual
// obs_pinned_joints list. Start is refused while a leg motor is offline.
//
// States:
//   IDLE     nothing is published (the bridge damps the joints)
//   HOMING   slow cosine move to home_pose, then HOLDING
//   HOLDING  the last target is held
//   RAMPING  policy running; the command blends from the start pose into the
//            policy targets over start_ramp_s
//   RUNNING  policy targets
// Models: the *.onnx files of models_dir (default: the folder of policy_path)
// are listed in ~/status for the GUI's dropdown, which loads one by setting
// policy_path (a model whose input/output sizes do not match is rejected and
// the current one stays loaded). While the policy runs, a new model is switched
// in live, but only at a standstill: zero requested velocity, body angular
// velocity <= switch_max_ang_vel and leg joint velocities <= switch_max_joint_vel
// for switch_settle_s. The switch restarts the last-action observation and
// blends from the last target into the new policy over start_ramp_s (RAMPING),
// like Start. ~/status reports switch_ok / switch_reason for the GUI.
//
// The policy can only be started while the IMU reports the robot upright
// (tilt <= max_start_tilt_deg). The tilt is computed continuously from the IMU
// and published in ~/status so the GUI can disable Start.
//
// Wireless remote: with use_remote the left stick commands vx (up/down) and vy
// (left/right) and the right stick wz (left/right), scaled so that full
// deflection reaches the velocity limits (remote_deadzone gives an exact 0).
// With remote_start, pressing Start on the remote runs the same Start as the
// ~/start service, with all its checks (upright, leg motors online, fresh
// data, model loaded); a refused press is logged and reported in ~/status.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "getup/observation.hpp"
#include "getup/onnx_policy.hpp"
#include "getup/wireless_remote.hpp"
#include "ppowalk/motion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
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
    // Folder of selectable models (GUI dropdown); empty: the folder of
    // policy_path (~/ppowalk_models without one).
    models_dir_param_ = declare_parameter<std::string>("models_dir", "");
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
    // Calibration: added to the command (GUI or cmd_vel) to cancel the drift of
    // the real robot (e.g. walking slightly left with vy = 0). Applied only
    // while motion is commanded unless offsets_when_standing.
    cmd_offset_ = {declare_parameter<double>("cmd_vx_offset", 0.0),
      declare_parameter<double>("cmd_vy_offset", 0.0),
      declare_parameter<double>("cmd_wz_offset", 0.0)};
    offsets_when_standing_ = declare_parameter<bool>("offsets_when_standing", false);
    use_cmd_vel_ = declare_parameter<bool>("use_cmd_vel", false);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);
    // Wireless remote: sticks -> velocity command (use_remote), Start button ->
    // Start (remote_start).
    use_remote_ = declare_parameter<bool>("use_remote", true);
    remote_start_ = declare_parameter<bool>("remote_start", true);
    remote_deadzone_ = declare_parameter<double>("remote_deadzone", 0.1);
    remote_timeout_ = declare_parameter<double>("remote_timeout_s", 0.5);
    // Safety / motion.
    max_start_tilt_deg_ = declare_parameter<double>("max_start_tilt_deg", 30.0);
    start_ramp_s_ = declare_parameter<double>("start_ramp_s", 1.0);
    // Standstill required to switch the running policy (policy_path).
    switch_max_ang_vel_ = declare_parameter<double>("switch_max_ang_vel", 0.3);
    switch_max_joint_vel_ = declare_parameter<double>("switch_max_joint_vel", 0.5);
    switch_settle_s_ = declare_parameter<double>("switch_settle_s", 0.5);
    home_pose_ = declare_parameter<std::vector<double>>("home_pose", std::vector<double>{});
    home_max_vel_ = declare_parameter<double>("home_max_vel", 0.3);
    command_joint_names_ = declare_parameter<std::vector<std::string>>(
      "command_joints", std::vector<std::string>{});
    always_commanded_ = declare_parameter<std::vector<std::string>>(
      "always_commanded_joints", std::vector<std::string>{"waist_yaw_joint"});
    held_targets_ =
      declare_parameter<std::vector<double>>("held_joint_targets", std::vector<double>{});
    pinned_names_ = declare_parameter<std::vector<std::string>>(
      "obs_pinned_joints", std::vector<std::string>{});
    detect_offline_ = declare_parameter<bool>("detect_offline", true);
    home_min_duration_ = declare_parameter<double>("home_min_duration_s", 2.0);
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu");
    const auto joint_states_topic =
      declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/ppowalk/joint_command");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto remote_topic = declare_parameter<std::string>("remote_topic", "/wireless_remote");
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
    is_action_.assign(n, false);
    for (size_t idx : action_index_) {
      is_action_[idx] = true;
    }
    error = check_names(always_commanded_, "always_commanded_joints");
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    if (command_joint_names_.empty()) {
      command_joint_names_ = action_joint_names_;
      for (const auto & name : always_commanded_) {
        if (std::find(command_joint_names_.begin(), command_joint_names_.end(), name) ==
          command_joint_names_.end())
        {
          command_joint_names_.push_back(name);
        }
      }
    }
    if (held_targets_.empty()) {
      held_targets_.assign(n, 0.0);
    }
    error = check_command_joints(command_joint_names_);
    if (error.empty() && held_targets_.size() != n) {
      error = "held_joint_targets must have len(joint_names) entries";
    }
    if (error.empty()) {
      error = check_names(pinned_names_, "obs_pinned_joints");
    }
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    error = check_remote(use_remote_, use_cmd_vel_, remote_deadzone_, remote_timeout_);
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    apply_command_joints(command_joint_names_);
    manual_pin_ = name_mask(pinned_names_);
    offline_now_.assign(n, false);
    session_offline_.assign(n, false);
    obs_builder_ = make_observation_builder(default_joint_pos_, observation_scales_, error);
    if (!obs_builder_) {
      throw std::invalid_argument(error);
    }
    if (num_obs > 0 && static_cast<size_t>(num_obs) != obs_builder_->size()) {
      throw std::invalid_argument(
              "num_obs=" + std::to_string(num_obs) + " but observation_terms give " +
              std::to_string(obs_builder_->size()));
    }
    models_dir_ = resolve_models_dir(models_dir_param_, policy_path_);
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
    policy_target_ = rest_targets();

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
    remote_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      remote_topic, 10,
      [this](sensor_msgs::msg::Joy::ConstSharedPtr msg) {on_remote(*msg);});

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
    scan_models();
    models_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {scan_models();});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_set_parameters(params);});

    RCLCPP_INFO(
      get_logger(), "ppowalk ready: %zu joints, %zu actions, %zu obs, policy %.0f Hz, start "
      "allowed below %.0f deg tilt. Model: %s", n, action_index_.size(), obs_builder_->size(),
      policy_rate_hz_, max_start_tilt_deg_, policy_path_.empty() ? "(none)" : policy_path_.c_str());
    RCLCPP_INFO(
      get_logger(), "Velocity command from %s; remote Start button %s (%s)",
      use_cmd_vel_ ? "cmd_vel" : use_remote_ ? "the wireless remote" : "the GUI",
      remote_start_ ? "enabled" : "disabled", remote_topic.c_str());
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

  std::string check_names(const std::vector<std::string> & names, const std::string & what) const
  {
    for (const auto & name : names) {
      if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
        return what + ": unknown joint '" + name + "'";
      }
    }
    return "";
  }

  std::string check_command_joints(const std::vector<std::string> & names) const
  {
    std::string error = check_names(names, "command_joints");
    if (!error.empty()) {
      return error;
    }
    for (const auto & a : action_joint_names_) {
      if (std::find(names.begin(), names.end(), a) == names.end()) {
        return "command_joints must include every action joint (missing " + a + ")";
      }
    }
    for (const auto & a : always_commanded_) {
      if (std::find(names.begin(), names.end(), a) == names.end()) {
        return "command_joints must include " + a + " (always_commanded_joints)";
      }
    }
    return "";
  }

  static std::string check_remote(
    bool use_remote, bool use_cmd_vel, double deadzone, double timeout)
  {
    if (use_remote && use_cmd_vel) {
      return "use_remote and use_cmd_vel are exclusive (one velocity command source)";
    }
    if (deadzone < 0.0 || deadzone > 0.9) {
      return "remote_deadzone must be in [0, 0.9]";
    }
    if (timeout <= 0.0) {
      return "remote_timeout_s must be > 0";
    }
    return "";
  }

  // models_dir, or the folder of the given policy path if empty; "~" expanded.
  static std::string resolve_models_dir(std::string dir, const std::string & policy_path)
  {
    if (dir.empty()) {
      dir = policy_path.empty() ? "~/ppowalk_models" :
        std::filesystem::path(policy_path).parent_path().string();
    }
    if (!dir.empty() && dir[0] == '~') {
      const char * home = std::getenv("HOME");
      dir = std::string(home ? home : "") + dir.substr(1);
    }
    dir = std::filesystem::path(dir).lexically_normal().string();
    while (dir.size() > 1 && dir.back() == '/') {
      dir.pop_back();
    }
    return dir;
  }

  // *.onnx files of models_dir, sorted (offered by the GUI's model dropdown).
  void scan_models()
  {
    std::vector<std::string> names;
    std::error_code ec;
    std::filesystem::directory_iterator it(models_dir_, ec);
    models_error_ = ec ? ec.message() : "";
    for (; !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
      std::error_code file_ec;
      const auto name = it->path().filename().string();
      if (it->path().extension() == ".onnx" && it->is_regular_file(file_ec) &&
        name.find_first_of("\"\\") == std::string::npos)
      {
        names.push_back(name);
      }
    }
    std::sort(names.begin(), names.end());
    models_ = std::move(names);
  }

  static std::string json_safe(const std::string & text)
  {
    std::string out;
    for (char ch : text) {
      if (ch != '"' && ch != '\\') {
        out += ch;
      }
    }
    return out;
  }

  std::vector<bool> name_mask(const std::vector<std::string> & names) const
  {
    std::vector<bool> mask(joint_names_.size(), false);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      mask[i] = std::find(names.begin(), names.end(), joint_names_[i]) != names.end();
    }
    return mask;
  }

  void apply_command_joints(const std::vector<std::string> & names)
  {
    command_joint_names_ = names;
    commanded_ = name_mask(names);
    command_index_.clear();
    command_msg_.name.clear();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (commanded_[i]) {
        command_index_.push_back(i);
        command_msg_.name.push_back(joint_names_[i]);
      }
    }
    command_msg_.position.assign(command_index_.size(), 0.0);
  }

  // Targets with zero action: legs at the default pose, the rest at their
  // hold targets.
  std::vector<double> rest_targets() const
  {
    std::vector<double> t(joint_names_.size());
    for (size_t i = 0; i < t.size(); ++i) {
      t[i] = is_action_[i] ? default_joint_pos_[i] : held_targets_[i];
    }
    return t;
  }

  bool running() const
  {
    return mode_ == Mode::kRamping || mode_ == Mode::kRunning;
  }

  // Joints whose observation is reported at the hold pose, at rest.
  std::vector<bool> pinned_mask() const
  {
    std::vector<bool> pin(joint_names_.size());
    for (size_t i = 0; i < pin.size(); ++i) {
      const bool offline = detect_offline_ && (running() ? session_offline_[i] : offline_now_[i]);
      pin[i] = manual_pin_[i] || !commanded_[i] || offline;
    }
    return pin;
  }

  std::vector<std::string> names_where(const std::vector<bool> & mask) const
  {
    std::vector<std::string> out;
    for (size_t i = 0; i < mask.size(); ++i) {
      if (mask[i]) {
        out.push_back(joint_names_[i]);
      }
    }
    return out;
  }

  static std::string json_list(const std::vector<std::string> & names)
  {
    std::string out = "[";
    for (const auto & n : names) {
      out += (out.size() > 1 ? ",\"" : "\"") + n + "\"";
    }
    return out + "]";
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
      "imu_topic", "joint_states_topic", "command_topic", "cmd_vel_topic", "publish_debug",
      "always_commanded_joints", "remote_topic"};
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    auto action_scale = action_scale_;
    auto default_joint_pos = default_joint_pos_;
    auto observation_scales = observation_scales_;
    auto home_pose = home_pose_;
    auto cmd = cmd_;
    auto cmd_min = cmd_min_;
    auto cmd_max = cmd_max_;
    auto cmd_offset = cmd_offset_;
    bool offsets_when_standing = offsets_when_standing_;
    double action_clip = action_clip_, data_timeout = data_timeout_;
    double policy_rate = policy_rate_hz_, command_rate = command_rate_hz_;
    double max_tilt = max_start_tilt_deg_, ramp = start_ramp_s_;
    double switch_ang = switch_max_ang_vel_, switch_joint = switch_max_joint_vel_;
    double switch_settle = switch_settle_s_;
    double home_vel = home_max_vel_, home_min = home_min_duration_;
    double cmd_vel_timeout = cmd_vel_timeout_;
    bool use_cmd_vel = use_cmd_vel_;
    bool use_remote = use_remote_, remote_start = remote_start_;
    double remote_deadzone = remote_deadzone_, remote_timeout = remote_timeout_;
    auto command_joints = command_joint_names_;
    auto held_targets = held_targets_;
    auto pinned_names = pinned_names_;
    bool detect_offline = detect_offline_;
    bool command_joints_changed = false;
    std::optional<std::string> policy_path;
    std::optional<std::string> models_dir;
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
        } else if (name == "switch_max_ang_vel") {
          switch_ang = p.as_double();
        } else if (name == "switch_max_joint_vel") {
          switch_joint = p.as_double();
        } else if (name == "switch_settle_s") {
          switch_settle = p.as_double();
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
        } else if (name == "cmd_vx_offset") {
          cmd_offset[0] = p.as_double();
        } else if (name == "cmd_vy_offset") {
          cmd_offset[1] = p.as_double();
        } else if (name == "cmd_wz_offset") {
          cmd_offset[2] = p.as_double();
        } else if (name == "offsets_when_standing") {
          offsets_when_standing = p.as_bool();
        } else if (name == "use_cmd_vel") {
          use_cmd_vel = p.as_bool();
        } else if (name == "cmd_vel_timeout_s") {
          cmd_vel_timeout = p.as_double();
        } else if (name == "use_remote") {
          use_remote = p.as_bool();
        } else if (name == "remote_start") {
          remote_start = p.as_bool();
        } else if (name == "remote_deadzone") {
          remote_deadzone = p.as_double();
        } else if (name == "remote_timeout_s") {
          remote_timeout = p.as_double();
        } else if (name == "policy_path") {
          policy_path = p.as_string();
        } else if (name == "models_dir") {
          models_dir = p.as_string();
        } else if (name == "command_joints") {
          command_joints = p.as_string_array();
          result.reason = check_command_joints(command_joints);
          command_joints_changed = command_joints != command_joint_names_;
        } else if (name == "held_joint_targets") {
          held_targets = p.as_double_array();
          if (held_targets.size() != joint_names_.size()) {
            result.reason = "held_joint_targets must have len(joint_names) entries";
          }
        } else if (name == "obs_pinned_joints") {
          pinned_names = p.as_string_array();
          result.reason = check_names(pinned_names, "obs_pinned_joints");
        } else if (name == "detect_offline") {
          detect_offline = p.as_bool();
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
    if (switch_ang <= 0.0 || switch_joint <= 0.0 || switch_settle < 0.0) {
      result.reason = "switch_max_ang_vel, switch_max_joint_vel > 0; switch_settle_s >= 0";
      return result;
    }
    if (max_tilt <= 0.0 || max_tilt > 90.0) {
      result.reason = "max_start_tilt_deg must be in (0, 90]";
      return result;
    }
    result.reason = check_remote(use_remote, use_cmd_vel, remote_deadzone, remote_timeout);
    if (!result.reason.empty()) {
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
    if (command_joints_changed && running()) {
      result.reason = "command_joints can only be changed while the policy is stopped";
      return result;
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
      if (running()) {
        const std::string blocker = switch_blocker();
        if (!blocker.empty()) {
          result.reason = "policy switch refused: " + blocker;
          return result;
        }
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
    cmd_offset_ = cmd_offset;
    offsets_when_standing_ = offsets_when_standing;
    action_clip_ = action_clip;
    data_timeout_ = data_timeout;
    max_start_tilt_deg_ = max_tilt;
    start_ramp_s_ = ramp;
    switch_max_ang_vel_ = switch_ang;
    switch_max_joint_vel_ = switch_joint;
    switch_settle_s_ = switch_settle;
    home_max_vel_ = home_vel;
    home_min_duration_ = home_min;
    use_cmd_vel_ = use_cmd_vel;
    cmd_vel_timeout_ = cmd_vel_timeout;
    if (use_remote != use_remote_ || remote_start != remote_start_) {
      RCLCPP_WARN(
        get_logger(), "Wireless remote: velocity command %s, Start button %s",
        use_remote ? "ON" : "off", remote_start ? "ON" : "off");
    }
    use_remote_ = use_remote;
    remote_start_ = remote_start;
    remote_deadzone_ = remote_deadzone;
    remote_timeout_ = remote_timeout;
    held_targets_ = std::move(held_targets);
    pinned_names_ = std::move(pinned_names);
    manual_pin_ = name_mask(pinned_names_);
    detect_offline_ = detect_offline;
    if (command_joints_changed) {
      apply_command_joints(command_joints);
      RCLCPP_WARN(get_logger(), "Now commanding %zu joints", command_index_.size());
    }
    if (obs_builder) {
      default_joint_pos_ = std::move(default_joint_pos);
      observation_scales_ = std::move(observation_scales);
      obs_builder_ = std::move(obs_builder);
    }
    if (policy) {
      policy_ = std::move(policy);
      policy_path_ = *policy_path;
      if (running()) {
        // Live switch (standstill checked above): restart the last-action
        // observation and blend from the last target into the new policy.
        std::fill(action_.begin(), action_.end(), 0.0f);
        move_from_ = current_pose();
        move_start_ = SteadyClock::now();
        mode_ = start_ramp_s_ > 0.0 ? Mode::kRamping : Mode::kRunning;
        step_ = 0;
        still_since_.reset();
        RCLCPP_WARN(get_logger(), "Switched the running policy to %s", policy_path_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "Loaded policy %s", policy_path_.c_str());
      }
    }
    if (models_dir) {
      models_dir_param_ = *models_dir;
      models_dir_ = resolve_models_dir(models_dir_param_, policy_path_);
      scan_models();
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
    const bool has_eff = msg.effort.size() == msg.name.size();
    const bool session = running();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const size_t k = js_index_[i];
      state_.joint_pos[i] = msg.position[k];
      state_.joint_vel[i] = has_vel ? msg.velocity[k] : 0.0;
      // The bridge forwards offline motors as exactly q = dq = tau = 0, which
      // a live motor practically never reports.
      offline_now_[i] = has_vel && has_eff && msg.position[k] == 0.0 && msg.velocity[k] == 0.0 &&
        msg.effort[k] == 0.0;
      if (session && offline_now_[i]) {
        session_offline_[i] = true;
      }
    }
    last_js_ = SteadyClock::now();
  }

  // Wireless remote (sensor_msgs/Joy from the bridge, Unitree button order).
  void on_remote(const sensor_msgs::msg::Joy & msg)
  {
    using getup::WirelessRemote;
    if (msg.axes.size() < 4 || msg.buttons.size() < WirelessRemote::kNumButtons) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Remote message needs >= 4 axes and %d buttons",
        static_cast<int>(WirelessRemote::kNumButtons));
      return;
    }
    remote_axes_ = {msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3]};
    remote_buttons_ = 0;
    for (int b = 0; b < WirelessRemote::kNumButtons; ++b) {
      if (msg.buttons[b]) {
        remote_buttons_ |= 1u << b;
      }
    }
    last_remote_ = SteadyClock::now();
    // Start acts on the press (rising edge) only. A button already held when
    // the first message arrives does not count.
    const bool start_down = msg.buttons[WirelessRemote::kStart] != 0;
    const bool pressed = remote_start_down_ && !*remote_start_down_ && start_down;
    remote_start_down_ = start_down;
    if (!pressed) {
      return;
    }
    if (!remote_start_) {
      remote_event("Start button ignored (remote_start is off)");
      return;
    }
    RCLCPP_INFO(get_logger(), "Remote: Start button pressed");
    std::string message;
    const bool ok = start(message);
    remote_event(ok ? "Start button: policy started" : "Start button: refused (" + message + ")");
  }

  void remote_event(const std::string & text)
  {
    remote_event_ = json_safe(text);
    last_remote_event_ = SteadyClock::now();
    ++remote_events_;
  }

  static double age(const std::optional<SteadyClock::time_point> & t)
  {
    return t ? std::chrono::duration<double>(SteadyClock::now() - *t).count() : -1.0;
  }

  bool remote_fresh() const
  {
    return last_remote_ && age(last_remote_) < remote_timeout_;
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

  // Requested command: cmd_vel or the remote sticks (0 while their data is
  // stale), else the GUI parameters.
  std::array<double, 3> raw_command() const
  {
    if (use_cmd_vel_) {
      return (last_cmd_vel_ && age(last_cmd_vel_) < cmd_vel_timeout_) ?
             cmd_vel_ : std::array<double, 3>{0.0, 0.0, 0.0};
    }
    if (use_remote_) {
      if (!remote_fresh()) {
        return {0.0, 0.0, 0.0};
      }
      // Stick up = forward, stick right = -y (robot y points left) and a
      // clockwise turn (-wz).
      const double lx = remote_axes_[0], ly = remote_axes_[1], rx = remote_axes_[2];
      return {stick_command(ly, remote_deadzone_, cmd_min_[0], cmd_max_[0]),
        stick_command(-lx, remote_deadzone_, cmd_min_[1], cmd_max_[1]),
        stick_command(-rx, remote_deadzone_, cmd_min_[2], cmd_max_[2])};
    }
    return cmd_;
  }

  std::string command_source() const
  {
    if (use_cmd_vel_) {
      return last_cmd_vel_ && age(last_cmd_vel_) < cmd_vel_timeout_ ? "cmd_vel" :
             "cmd_vel (stale: 0)";
    }
    if (use_remote_) {
      return remote_fresh() ? "remote" : "remote (no data: 0)";
    }
    return "gui";
  }

  // Command fed to the policy: requested + calibration offsets (only while
  // motion is requested, unless offsets_when_standing), clamped to the limits.
  std::array<double, 3> velocity_command() const
  {
    std::array<double, 3> c = raw_command();
    const bool standing = c[0] == 0.0 && c[1] == 0.0 && c[2] == 0.0;
    for (int k = 0; k < 3; ++k) {
      if (!standing || offsets_when_standing_) {
        c[k] += cmd_offset_[k];
      }
      c[k] = std::clamp(c[k], cmd_min_[k], cmd_max_[k]);
    }
    return c;
  }

  // Why the requested command / the robot is not at rest ("" when it is).
  std::string motion_blocker() const
  {
    const auto c = raw_command();
    if (std::abs(c[0]) > 1e-3 || std::abs(c[1]) > 1e-3 || std::abs(c[2]) > 1e-3) {
      return "velocity command not zero";
    }
    char buf[96];
    const auto & w = state_.base_ang_vel;
    const double ang = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (ang > switch_max_ang_vel_) {
      std::snprintf(
        buf, sizeof(buf), "body rotating (%.2f > %.2f rad/s)", ang, switch_max_ang_vel_);
      return buf;
    }
    double joint = 0.0;
    for (size_t idx : action_index_) {
      joint = std::max(joint, std::abs(state_.joint_vel[idx]));
    }
    if (joint > switch_max_joint_vel_) {
      std::snprintf(
        buf, sizeof(buf), "legs moving (%.2f > %.2f rad/s)", joint, switch_max_joint_vel_);
      return buf;
    }
    return "";
  }

  // Every policy step: (re)start or reset the standstill timer.
  void update_standstill()
  {
    if (!motion_blocker().empty()) {
      still_since_.reset();
    } else if (!still_since_) {
      still_since_ = SteadyClock::now();
    }
  }

  // Why the running policy cannot be switched now ("" when it can).
  std::string switch_blocker() const
  {
    if (mode_ == Mode::kRamping) {
      return "ramping, wait until RUNNING";
    }
    if (mode_ != Mode::kRunning) {
      return "policy not running";
    }
    if (!data_fresh()) {
      return "no fresh IMU/JointState data";
    }
    const std::string motion = motion_blocker();
    if (!motion.empty()) {
      return motion;
    }
    if (!still_since_ || age(still_since_) < switch_settle_s_) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "settling (%.1f s at rest needed)", switch_settle_s_);
      return buf;
    }
    return "";
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
    } else if (detect_offline_ && [this]() {
        for (size_t idx : action_index_) {
          if (offline_now_[idx]) {
            return true;
          }
        }
        return false;
      }())
    {
      std::string legs;
      for (size_t idx : action_index_) {
        if (offline_now_[idx]) {
          legs += (legs.empty() ? "" : ", ") + joint_names_[idx];
        }
      }
      message = "leg motor offline: " + legs;
    } else if (!upright()) {
      char buf[96];
      std::snprintf(
        buf, sizeof(buf), "robot not upright: tilt %.1f deg > %.1f deg", tilt_deg_,
        max_start_tilt_deg_);
      message = buf;
    } else {
      std::fill(action_.begin(), action_.end(), 0.0f);
      policy_target_ = rest_targets();
      session_offline_ = offline_now_;
      move_from_ = current_pose();
      move_start_ = SteadyClock::now();
      mode_ = start_ramp_s_ > 0.0 ? Mode::kRamping : Mode::kRunning;
      step_ = 0;
      still_since_.reset();
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
    update_standstill();
    state_.command = velocity_command();
    obs_state_ = state_;
    const auto pin = pinned_mask();
    for (size_t i = 0; i < pin.size(); ++i) {
      if (pin[i]) {
        obs_state_.joint_pos[i] = is_action_[i] ? default_joint_pos_[i] : held_targets_[i];
        obs_state_.joint_vel[i] = 0.0;
      }
    }
    const auto t0 = SteadyClock::now();
    obs_builder_->build(obs_state_, action_, obs_);
    const auto & out = policy_->infer(obs_);
    inference_ms_ = std::chrono::duration<double, std::milli>(SteadyClock::now() - t0).count();
    auto target = rest_targets();
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
    for (size_t k = 0; k < command_index_.size(); ++k) {
      command_msg_.position[k] = (*target_)[command_index_[k]];
    }
    command_msg_.header.stamp = this->now();
    command_pub_->publish(command_msg_);
  }

  void publish_status()
  {
    const auto c = velocity_command();
    const std::string path = json_safe(policy_path_);
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
      command_source().c_str());
    std::string json = buf;
    json.pop_back();
    const auto raw = raw_command();
    char extra[160];
    std::snprintf(
      extra, sizeof(extra), ",\"command_raw\":[%.3f,%.3f,%.3f],\"command_offset\":[%.3f,%.3f,%.3f]",
      raw[0], raw[1], raw[2], cmd_offset_[0], cmd_offset_[1], cmd_offset_[2]);
    json += extra;
    const std::string blocker = switch_blocker();
    json += std::string(",\"switch_ok\":") + (blocker.empty() ? "true" : "false") +
      ",\"switch_reason\":\"" + json_safe(blocker) + "\"";
    json += ",\"command_joints\":" + json_list(command_joint_names_) +
      ",\"pinned_joints\":" + json_list(names_where(pinned_mask())) +
      ",\"offline_detected\":" + json_list(names_where(offline_now_)) + ",\"remote\":" +
      remote_status() + ",\"models_dir\":\"" + json_safe(models_dir_) + "\",\"models\":" +
      json_list(models_) + ",\"models_error\":\"" + json_safe(models_error_) + "\"}";
    std_msgs::msg::String msg;
    msg.data = json;
    status_pub_->publish(msg);
  }

  std::string remote_status() const
  {
    std::vector<std::string> buttons;
    for (int b = 0; b < getup::WirelessRemote::kNumButtons; ++b) {
      if (remote_buttons_ & (1u << b)) {
        buttons.push_back(getup::remote_button_name(b));
      }
    }
    char buf[384];
    std::snprintf(
      buf, sizeof(buf),
      "{\"use_remote\":%s,\"remote_start\":%s,\"fresh\":%s,\"age\":%.3f,"
      "\"sticks\":[%.3f,%.3f,%.3f,%.3f],\"events\":%d,\"event_age\":%.1f,\"last_event\":\"",
      use_remote_ ? "true" : "false", remote_start_ ? "true" : "false",
      remote_fresh() ? "true" : "false", age(last_remote_), remote_axes_[0], remote_axes_[1],
      remote_axes_[2], remote_axes_[3], remote_events_, age(last_remote_event_));
    return buf + remote_event_ + "\",\"buttons\":" +
           json_list(remote_fresh() ? buttons : std::vector<std::string>{}) + "}";
  }

  // Parameters.
  std::string policy_path_;
  std::string models_dir_param_;
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
  std::array<double, 3> cmd_offset_{0.0, 0.0, 0.0};
  bool offsets_when_standing_{false};
  bool use_cmd_vel_{false};
  double cmd_vel_timeout_{0.5};
  bool use_remote_{true};
  bool remote_start_{true};
  double remote_deadzone_{0.1};
  double remote_timeout_{0.5};
  double max_start_tilt_deg_{30.0};
  double start_ramp_s_{1.0};
  double switch_max_ang_vel_{0.3};
  double switch_max_joint_vel_{0.5};
  double switch_settle_s_{0.5};
  std::vector<double> home_pose_;
  std::vector<std::string> command_joint_names_;
  std::vector<std::string> always_commanded_;
  std::vector<double> held_targets_;
  std::vector<std::string> pinned_names_;
  bool detect_offline_{true};
  double home_max_vel_{0.3};
  double home_min_duration_{2.0};

  // State.
  std::string models_dir_;  // resolved models_dir
  std::vector<std::string> models_;  // *.onnx in models_dir_
  std::string models_error_;
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
  std::optional<SteadyClock::time_point> still_since_;  // at rest since (update_standstill)
  getup::RobotState state_;
  getup::RobotState obs_state_;  // state_ with pinned joints replaced
  std::vector<bool> is_action_;
  std::vector<bool> commanded_;
  std::vector<size_t> command_index_;
  std::vector<bool> manual_pin_;
  std::vector<bool> offline_now_;
  std::vector<bool> session_offline_;
  std::vector<std::string> last_js_names_;
  std::vector<size_t> js_index_;
  std::optional<SteadyClock::time_point> last_imu_;
  std::optional<SteadyClock::time_point> last_js_;
  std::array<double, 3> cmd_vel_{0.0, 0.0, 0.0};
  std::optional<SteadyClock::time_point> last_cmd_vel_;
  std::array<double, 4> remote_axes_{0.0, 0.0, 0.0, 0.0};  // lx, ly, rx, ry
  uint32_t remote_buttons_{0};
  std::optional<SteadyClock::time_point> last_remote_;
  std::optional<bool> remote_start_down_;  // Start button state in the last message
  std::string remote_event_;
  std::optional<SteadyClock::time_point> last_remote_event_;
  int remote_events_{0};

  // ROS interfaces.
  sensor_msgs::msg::JointState command_msg_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr remote_sub_;
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
  rclcpp::TimerBase::SharedPtr models_timer_;
};

}  // namespace ppowalk

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ppowalk::PpowalkPolicyNode>());
  rclcpp::shutdown();
  return 0;
}
