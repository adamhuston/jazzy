#include "rov2_core/core_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_loader.hpp"

#include "rov2_core/qos_profiles.hpp"

namespace rov2_core
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using SystemStatus = rov2_interfaces::msg::SystemStatus;
using PluginStatus = rov2_interfaces::msg::PluginStatus;
using ModeCommand = rov2_interfaces::msg::ModeCommand;

CoreNode::CoreNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("rov2_core", options)
{
  declare_parameter<double>("loop_rate_hz", 10.0);
  declare_parameter<double>("cmd_vel_timeout_sec", 0.5);
  declare_parameter<bool>("autostart", true);
  declare_parameter<std::vector<std::string>>("sensor_plugins", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("actuator_plugins", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("brain_plugins", std::vector<std::string>{});

  sensor_loader_ = std::make_unique<pluginlib::ClassLoader<SensorPlugin>>(
    "rov2_core", "rov2_core::SensorPlugin");
  actuator_loader_ = std::make_unique<pluginlib::ClassLoader<ActuatorPlugin>>(
    "rov2_core", "rov2_core::ActuatorPlugin");
  brain_loader_ = std::make_unique<pluginlib::ClassLoader<BrainPlugin>>(
    "rov2_core", "rov2_core::BrainPlugin");
}

template<typename PluginT>
void CoreNode::load_plugins(
  const std::string & param_name,
  pluginlib::ClassLoader<PluginT> & loader,
  std::vector<std::shared_ptr<PluginT>> & out)
{
  const auto names = get_parameter(param_name).as_string_array();
  for (const auto & lookup_name : names) {
    try {
      auto plugin = loader.createSharedInstance(lookup_name);
      plugin->set_plugin_type(lookup_name);
      if (!plugin->on_init(shared_from_this(), lookup_name)) {
        RCLCPP_ERROR(get_logger(), "Plugin '%s' failed on_init; skipping", lookup_name.c_str());
        continue;
      }
      if (!plugin->on_configure()) {
        RCLCPP_ERROR(
          get_logger(), "Plugin '%s' failed on_configure; skipping", lookup_name.c_str());
        continue;
      }
      out.push_back(plugin);
      RCLCPP_INFO(get_logger(), "Loaded %s plugin '%s'", param_name.c_str(), lookup_name.c_str());
    } catch (const pluginlib::PluginlibException & ex) {
      // Missing/optional plugins are non-fatal: log actionable detail and continue.
      RCLCPP_ERROR(
        get_logger(), "Failed to load plugin '%s' (%s): %s",
        lookup_name.c_str(), param_name.c_str(), ex.what());
    }
  }
}

CallbackReturn CoreNode::on_configure(const rclcpp_lifecycle::State &)
{
  loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();
  if (loop_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "loop_rate_hz must be > 0 (got %f)", loop_rate_hz_);
    return CallbackReturn::FAILURE;
  }
  loop_period_ms_ = 1000.0 / loop_rate_hz_;
  cmd_vel_timeout_sec_ = get_parameter("cmd_vel_timeout_sec").as_double();

  status_pub_ = create_publisher<SystemStatus>("~/status", qos::status());
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", qos::status());

  mode_sub_ = create_subscription<ModeCommand>(
    "~/mode_command", qos::control(),
    std::bind(&CoreNode::on_mode_command, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", qos::control(),
    std::bind(&CoreNode::on_cmd_vel, this, std::placeholders::_1));
  set_mode_srv_ = create_service<rov2_interfaces::srv::SetMode>(
    "~/set_mode",
    std::bind(&CoreNode::on_set_mode, this, std::placeholders::_1, std::placeholders::_2));

  load_plugins<SensorPlugin>("sensor_plugins", *sensor_loader_, sensors_);
  load_plugins<ActuatorPlugin>("actuator_plugins", *actuator_loader_, actuators_);
  load_plugins<BrainPlugin>("brain_plugins", *brain_loader_, brains_);

  RCLCPP_INFO(
    get_logger(), "Configured: %zu sensor, %zu actuator, %zu brain plugin(s)",
    sensors_.size(), actuators_.size(), brains_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreNode::on_activate(const rclcpp_lifecycle::State &)
{
  status_pub_->on_activate();
  diag_pub_->on_activate();

  for (auto & p : sensors_) {p->on_activate();}
  for (auto & p : actuators_) {p->on_activate();}
  for (auto & p : brains_) {p->on_activate();}

  mode_ = ModeCommand::ACTIVE;
  loop_count_ = 0;
  have_last_tick_ = false;
  have_cmd_ = false;

  const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
  alive_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CoreNode::alive_loop, this));

  RCLCPP_INFO(get_logger(), "Activated alive loop at %.1f Hz", loop_rate_hz_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (alive_timer_) {
    alive_timer_->cancel();
    alive_timer_.reset();
  }
  for (auto & p : brains_) {p->on_deactivate();}
  for (auto & p : actuators_) {p->on_deactivate();}
  for (auto & p : sensors_) {p->on_deactivate();}

  diag_pub_->on_deactivate();
  status_pub_->on_deactivate();

  mode_ = ModeCommand::STANDBY;
  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (auto & p : brains_) {p->on_cleanup();}
  for (auto & p : actuators_) {p->on_cleanup();}
  for (auto & p : sensors_) {p->on_cleanup();}
  brains_.clear();
  actuators_.clear();
  sensors_.clear();

  set_mode_srv_.reset();
  cmd_vel_sub_.reset();
  mode_sub_.reset();
  diag_pub_.reset();
  status_pub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CoreNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  for (auto & p : brains_) {p->on_shutdown();}
  for (auto & p : actuators_) {p->on_shutdown();}
  for (auto & p : sensors_) {p->on_shutdown();}
  return CallbackReturn::SUCCESS;
}

void CoreNode::alive_loop()
{
  const rclcpp::Time now = this->now();
  if (have_last_tick_) {
    const double elapsed_ms = (now - last_tick_).seconds() * 1000.0;
    last_jitter_ms_ = elapsed_ms - loop_period_ms_;
  }
  last_tick_ = now;
  have_last_tick_ = true;
  ++loop_count_;

  for (auto & s : sensors_) {s->poll();}
  for (auto & b : brains_) {b->think();}

  geometry_msgs::msg::Twist outgoing;
  if (mode_ == ModeCommand::ACTIVE) {
    const bool cmd_fresh = have_cmd_ &&
      (now - last_cmd_time_).seconds() <= cmd_vel_timeout_sec_;
    if (cmd_fresh) {
      outgoing = last_cmd_;
    } else if (have_cmd_) {
      // Command went stale: fail safe to zero motion and flag the gap.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "cmd_vel stale (> %.3fs); holding zero motion", cmd_vel_timeout_sec_);
    }
  }  // STANDBY/SAFE hold zero motion.
  for (auto & a : actuators_) {a->apply_command(outgoing);}

  const auto status = build_status();
  if (status_pub_ && status_pub_->is_activated()) {
    status_pub_->publish(status);
  }
  publish_diagnostics(status);
}

SystemStatus CoreNode::build_status()
{
  SystemStatus msg;
  msg.stamp = this->now();
  msg.loop_count = loop_count_;
  msg.loop_period_ms = loop_period_ms_;
  msg.loop_jitter_ms = last_jitter_ms_;

  bool any_fault = false;
  bool any_degraded = false;
  auto append = [&](const std::shared_ptr<PluginBase> & p) {
      const auto s = p->get_status();
      if (s.state == PluginStatus::FAULT) {any_fault = true;}
      if (s.state == PluginStatus::DEGRADED) {any_degraded = true;}
      msg.plugins.push_back(s);
    };
  for (auto & p : sensors_) {append(p);}
  for (auto & p : actuators_) {append(p);}
  for (auto & p : brains_) {append(p);}

  if (any_fault) {
    msg.state = SystemStatus::FAULT;
    msg.state_label = "fault";
  } else if (any_degraded) {
    msg.state = SystemStatus::DEGRADED;
    msg.state_label = "degraded";
  } else {
    msg.state = SystemStatus::ACTIVE;
    msg.state_label = "active";
  }
  return msg;
}

void CoreNode::publish_diagnostics(const SystemStatus & status)
{
  if (!diag_pub_ || !diag_pub_->is_activated()) {
    return;
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = status.stamp;

  diagnostic_msgs::msg::DiagnosticStatus core;
  core.name = "rov2/core";
  core.hardware_id = "rov2_core";
  core.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  core.message = status.state_label;
  diagnostic_msgs::msg::KeyValue kv_loops;
  kv_loops.key = "loop_count";
  kv_loops.value = std::to_string(status.loop_count);
  diagnostic_msgs::msg::KeyValue kv_jitter;
  kv_jitter.key = "loop_jitter_ms";
  kv_jitter.value = std::to_string(status.loop_jitter_ms);
  core.values.push_back(kv_loops);
  core.values.push_back(kv_jitter);
  array.status.push_back(core);

  for (const auto & p : status.plugins) {
    diagnostic_msgs::msg::DiagnosticStatus ds;
    ds.name = "rov2/" + p.category + "/" + p.name;
    ds.hardware_id = p.type;
    ds.message = p.message;
    switch (p.state) {
      case PluginStatus::FAULT:
        ds.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        break;
      case PluginStatus::DEGRADED:
        ds.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        break;
      case PluginStatus::ACTIVE:
        ds.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        break;
      default:
        ds.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        break;
    }
    array.status.push_back(ds);
  }
  diag_pub_->publish(array);
}

void CoreNode::on_mode_command(const ModeCommand::SharedPtr msg)
{
  std::string message;
  if (!apply_mode(msg->mode, msg->reason, message)) {
    RCLCPP_WARN(get_logger(), "Rejected mode command: %s", message.c_str());
  }
}

void CoreNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_ = *msg;
  last_cmd_time_ = this->now();
  have_cmd_ = true;
}

void CoreNode::on_set_mode(
  const std::shared_ptr<rov2_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rov2_interfaces::srv::SetMode::Response> response)
{
  response->success = apply_mode(request->mode, request->reason, response->message);
}

bool CoreNode::apply_mode(uint8_t mode, const std::string & reason, std::string & message)
{
  switch (mode) {
    case ModeCommand::STANDBY:
    case ModeCommand::ACTIVE:
    case ModeCommand::SAFE:
      break;
    default:
      message = "unknown mode value " + std::to_string(static_cast<int>(mode));
      return false;
  }
  mode_ = mode;
  if (mode == ModeCommand::SAFE) {
    last_cmd_ = geometry_msgs::msg::Twist();  // drop any pending motion
  }
  message = "mode set" + (reason.empty() ? std::string() : (" (" + reason + ")"));
  RCLCPP_INFO(get_logger(), "%s -> mode %d", message.c_str(), static_cast<int>(mode));
  return true;
}

}  // namespace rov2_core

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rov2_core::CoreNode)
