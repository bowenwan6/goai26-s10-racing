// s10_ros1_control: ROS 1 motion interface for the S10.
//
// ROS 1 in   cmd_sources (e.g. /rl_nav/cmd_vel, /cmd_vel)  geometry_msgs/Twist -> /NAV_CMD
//              only the ACTIVE source is forwarded ("source <name>" on /web_cmd selects it)
//            /rl_nav/gait_request std_msgs/String "flat" | "stairs": gait switch at a standstill
//            /web_cmd  std_msgs/String       "cmd4" stand, "cmd3" lie down,
//                                            "Nav stop" latch zero, "Nav continue",
//                                            "cmd1" navigation gait, "source <name>"
// ROS 1 out  /s10_control/state std_msgs/String JSON (5 Hz)
//            /s10_control/gait  std_msgs/String "flat" | "stairs" | "switching" | "none" (10 Hz)
// ROS 2 in   /MOTION_INFO drdds/MotionInfo (state, gait, measured velocity)
//            /NAV_CMD (monitor only): messages from any other publisher are counted
// ROS 2 out  /NAV_CMD, /MOTION_STATE, /GAIT  - only with --enable-motion
//
// Rules (S10 developer guide p.48-52 plus the 2026-09-08/17 field notes):
//   * Velocity is forwarded only in RL control (17) with a navigation gait
//     (0x3002/0x3003), with fresh feedback, fresh /cmd_vel, no latch, no fault and
//     no gait switch in progress. It is clamped and sent at a fixed rate; after any
//     stop, zero is sent for zero_hold_s and then nothing.
//   * Stand = MOTION_STATE 1 -> wait standing -> 17 -> wait RL -> GAIT nav gait
//     -> wait; every step waits for /MOTION_INFO confirmation.
//   * Gait switch (flat <-> stairs) only in RL: zero velocity -> measured still ->
//     /GAIT -> wait for /MOTION_INFO to confirm; timeout latches a stop.
//   * Lie down only after zero velocity and a measured still robot.
//   * "Nav stop" is a software stop (zero velocity), never the joint-damping soft
//     e-stop (state 2), which would drop the robot. The remote stays the e-stop.
//   * Another /NAV_CMD publisher that actually SENDS while we are armed latches a
//     fault (exclusive_mode messages). exclusive_mode publishers: any other
//     publisher's existence does (the shared dog always has two idle native ones).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "drdds/msg/gait.hpp"
#include "drdds/msg/motion_info.hpp"
#include "drdds/msg/motion_state.hpp"
#include "drdds/msg/nav_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

std::atomic<bool> g_stop{false};
void on_signal(int) {g_stop = true;}

double mono()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

int64_t wall_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string jstr(const std::string & s)
{
  std::ostringstream o;
  o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') {o << '\\' << c;} else if (c < 0x20) {o << ' ';} else {o << c;}
  }
  o << '"';
  return o.str();
}

std::string jnum(double v)
{
  if (!std::isfinite(v)) {return "null";}
  std::ostringstream o;
  o << std::setprecision(6) << v;
  return o.str();
}

std::string trim(std::string c)
{
  c.erase(0, c.find_first_not_of(" \t\r\n"));
  c.erase(c.find_last_not_of(" \t\r\n") + 1);
  return c;
}

constexpr int32_t kIdle = 0, kStanding = 1, kBootDamping = 3, kLying = 4, kRl = 17;
constexpr uint32_t kNavFlat = 0x3002, kNavStairs = 0x3003;
// Not a policy limit: the robot's own documented command range (developer guide 1.2.6: X +-1.67 m/s,
// Y +-0.4 m/s, yaw +-1.0 rad/s), with margin, so a typo in a config cannot send nonsense. The
// operator sets the real limit per run (config limits / --stage).
constexpr double kHardVx = 1.67, kHardVy = 0.5, kHardWz = 1.0;

// EXPERIMENT (2026-09-21): the gait used for "flat" can be replaced (stand.nav_gait / gait_switch.flat_gait), to try
// a gait the developer guide does not list for /GAIT (0xF002 踏步移动, 0x1002 高台, 0x1001 基础). Default 0x3002.
// If the robot does not confirm it on /MOTION_INFO the usual step timeout latches a stop: it fails safe.
uint32_t g_flat_gait = kNavFlat;

// Four gaits since 2026-09-21 (field-checked by the operator: /NAV_CMD drives the robot in all of them):
//   flat 0x3002 (navigation walk) | fast 0xF002 (踏步移动, quick) | stairs 0x3003 | platform 0x1002 (高台, jumps up a ledge)
constexpr uint32_t kFast = 0xF002, kPlatform = 0x1002;

const char * gait_name(uint32_t g)
{
  if (g == g_flat_gait) {return "flat";}
  return g == kNavFlat ? "flat" : g == kNavStairs ? "stairs" : g == kFast ? "fast" : g == kPlatform ? "platform" : "other";
}

struct Config
{
  std::vector<std::pair<std::string, std::string>> cmd_sources;  // name -> topic
  std::string cmd_source = "";
  std::string web_cmd_topic = "/web_cmd", state_topic = "/s10_control/state";
  std::string gait_request_topic = "/rl_nav/gait_request", gait_topic = "/s10_control/gait";
  double nav_rate = 10, cmd_timeout = 0.5, zero_hold = 1.0, info_timeout = 0.5;
  bool start_latched = false;
  bool stamp_robot = true;
  bool exclusive_messages = true;
  double max_vx = 0.3, max_vy = 0.1, max_wz = 0.5;
  uint32_t nav_gait = kNavFlat;
  double stand_settle = 2.0, stand_timeout = 15, resend = 3.0;
  double still_speed = 0.05, still_time = 1.0, still_timeout = 5.0, lie_timeout = 15;
  double gait_still_speed = 0.05, gait_still_time = 1.0, gait_still_timeout = 6.0, gait_timeout = 10.0;
  std::string w_stand = "cmd4", w_lie = "cmd3", w_stop = "Nav stop", w_continue = "Nav continue";
  std::string w_nav_mode = "cmd1", w_exit_nav = "cmd2", w_source = "source ";
};

Config load(const std::string & path)
{
  Config c;
  YAML::Node y = YAML::LoadFile(path);
  auto str = [&](const char * k, std::string & v) {if (y[k]) {v = y[k].as<std::string>();}};
  auto num = [](const YAML::Node & n, const char * k, double & v) {if (n && n[k]) {v = n[k].as<double>();}};
  if (y["cmd_sources"]) {
    for (const auto & kv : y["cmd_sources"]) {
      c.cmd_sources.emplace_back(kv.first.as<std::string>(), kv.second.as<std::string>());
    }
  }
  if (c.cmd_sources.empty()) {
    std::string topic = "/cmd_vel";
    str("cmd_vel_topic", topic);
    c.cmd_sources.emplace_back("default", topic);
  }
  str("cmd_source", c.cmd_source);
  if (c.cmd_source.empty()) {c.cmd_source = c.cmd_sources.front().first;}
  bool known = false;
  for (const auto & s : c.cmd_sources) {known = known || s.first == c.cmd_source;}
  if (!known) {throw std::runtime_error("cmd_source " + c.cmd_source + " is not in cmd_sources");}
  str("web_cmd_topic", c.web_cmd_topic);
  str("state_topic", c.state_topic);
  str("gait_request_topic", c.gait_request_topic);
  str("gait_topic", c.gait_topic);
  num(y, "nav_cmd_rate_hz", c.nav_rate);
  num(y, "cmd_vel_timeout_s", c.cmd_timeout);
  num(y, "zero_hold_s", c.zero_hold);
  num(y, "motion_info_timeout_s", c.info_timeout);
  if (y["start_latched"]) {c.start_latched = y["start_latched"].as<bool>();}
  if (y["stamp_source"]) {c.stamp_robot = y["stamp_source"].as<std::string>() != "local";}
  if (y["exclusive_mode"]) {
    const std::string m = y["exclusive_mode"].as<std::string>();
    if (m != "messages" && m != "publishers") {throw std::runtime_error("exclusive_mode must be messages or publishers");}
    c.exclusive_messages = m == "messages";
  }
  num(y["limits"], "max_vx", c.max_vx);
  num(y["limits"], "max_vy", c.max_vy);
  num(y["limits"], "max_wz", c.max_wz);
  if (y["stand"] && y["stand"]["nav_gait"]) {
    c.nav_gait = static_cast<uint32_t>(std::stoul(y["stand"]["nav_gait"].as<std::string>(), nullptr, 0));
  }
  num(y["stand"], "settle_s", c.stand_settle);
  num(y["stand"], "step_timeout_s", c.stand_timeout);
  num(y["stand"], "resend_period_s", c.resend);
  num(y["lie_down"], "still_speed", c.still_speed);
  num(y["lie_down"], "still_time_s", c.still_time);
  num(y["lie_down"], "wait_still_timeout_s", c.still_timeout);
  num(y["lie_down"], "step_timeout_s", c.lie_timeout);
  num(y["gait_switch"], "still_speed", c.gait_still_speed);
  num(y["gait_switch"], "still_time_s", c.gait_still_time);
  num(y["gait_switch"], "wait_still_timeout_s", c.gait_still_timeout);
  num(y["gait_switch"], "step_timeout_s", c.gait_timeout);
  if (YAML::Node w = y["web_cmd"]) {
    if (w["stand"]) {c.w_stand = w["stand"].as<std::string>();}
    if (w["lie_down"]) {c.w_lie = w["lie_down"].as<std::string>();}
    if (w["stop"]) {c.w_stop = w["stop"].as<std::string>();}
    if (w["continue"]) {c.w_continue = w["continue"].as<std::string>();}
    if (w["nav_mode"]) {c.w_nav_mode = w["nav_mode"].as<std::string>();}
    if (w["exit_nav_mode"]) {c.w_exit_nav = w["exit_nav_mode"].as<std::string>();}
    if (w["source_prefix"]) {c.w_source = w["source_prefix"].as<std::string>();}
  }
  auto in = [](double v, double lo, double hi) {return std::isfinite(v) && v >= lo && v <= hi;};
  if (!in(c.max_vx, 0, kHardVx) || !in(c.max_vy, 0, kHardVy) || !in(c.max_wz, 0, kHardWz)) {
    throw std::runtime_error("limits exceed the robot command range (1.67 m/s, 0.5 m/s, 1.0 rad/s)");
  }
  if (!in(c.nav_rate, 5, 50) || !in(c.cmd_timeout, 0.1, 2) || !in(c.zero_hold, 0.2, 5) ||
    !in(c.info_timeout, 0.1, 2))
  {
    throw std::runtime_error("rate/timeout values out of range");
  }
  if (y["gait_switch"] && y["gait_switch"]["flat_gait"]) {
    g_flat_gait = static_cast<uint32_t>(std::stoul(y["gait_switch"]["flat_gait"].as<std::string>(), nullptr, 0));
  }
  if (g_flat_gait != kNavFlat && g_flat_gait != 0xF002 && g_flat_gait != 0x1002 && g_flat_gait != 0x1001) {
    throw std::runtime_error("gait_switch.flat_gait must be 0x3002 (default), 0xF002, 0x1002 or 0x1001");
  }
  if (c.nav_gait != kNavFlat && c.nav_gait != kNavStairs && c.nav_gait != g_flat_gait) {
    throw std::runtime_error("stand.nav_gait must be 0x3002, 0x3003 or gait_switch.flat_gait");
  }
  return c;
}

enum class Seq {None, StandUp, StandSettle, StandRl, StandGait, LieStill, LieDown, GaitStill, GaitWait};

const char * seq_name(Seq s)
{
  switch (s) {
    case Seq::None: return "none";
    case Seq::StandUp: return "stand:wait_standing";
    case Seq::StandSettle: return "stand:settle";
    case Seq::StandRl: return "stand:wait_rl";
    case Seq::StandGait: return "stand:wait_nav_gait";
    case Seq::LieStill: return "lie:wait_still";
    case Seq::LieDown: return "lie:wait_lying";
    case Seq::GaitStill: return "gait:wait_still";
    case Seq::GaitWait: return "gait:wait_confirm";
  }
  return "?";
}

class Controller
{
public:
  Controller(const Config & cfg, bool enable_motion, const std::string & event_log)
  : cfg_(cfg), enable_(enable_motion), latched_(cfg.start_latched), source_(cfg.cmd_source)
  {
    if (!event_log.empty()) {events_.open(event_log, std::ios::app);}
    node_ = std::make_shared<rclcpp::Node>(
      "s10_ros1_control",
      rclcpp::NodeOptions().enable_rosout(false).start_parameter_services(false)
      .start_parameter_event_publisher(false));
    info_sub_ = node_->create_subscription<drdds::msg::MotionInfo>(
      "/MOTION_INFO", rclcpp::SensorDataQoS(),
      [this](drdds::msg::MotionInfo::ConstSharedPtr m) {on_info(*m);});
    // Monitor: who else sends /NAV_CMD. Best-effort/volatile matches any publisher QoS.
    nav_mon_sub_ = node_->create_subscription<drdds::msg::NavCmd>(
      "/NAV_CMD", rclcpp::SensorDataQoS().keep_last(20),
      [this](const drdds::msg::NavCmd &, const rclcpp::MessageInfo & info) {on_nav_seen(info);});
    ros::NodeHandle nh;
    for (const auto & s : cfg_.cmd_sources) {
      const std::string name = s.first;
      cmd_subs_.push_back(nh.subscribe<geometry_msgs::Twist>(
          s.second, 1, [this, name](const geometry_msgs::Twist::ConstPtr & m) {on_cmd(m, name);}));
      cmd_ignored_[name] = 0;
    }
    web_sub_ = nh.subscribe(cfg_.web_cmd_topic, 10, &Controller::on_web, this);
    gait_req_sub_ = nh.subscribe(cfg_.gait_request_topic, 5, &Controller::on_gait_request, this);
    state_pub_ = nh.advertise<std_msgs::String>(cfg_.state_topic, 1);
    gait_pub1_ = nh.advertise<std_msgs::String>(cfg_.gait_topic, 1);
    std::string srcs;
    for (const auto & s : cfg_.cmd_sources) {srcs += (srcs.empty() ? "" : ",") + s.first + "=" + s.second;}
    event("start", std::string("{\"enable_motion\":") + (enable_ ? "true" : "false") +
      ",\"max_vx\":" + jnum(cfg_.max_vx) + ",\"max_vy\":" + jnum(cfg_.max_vy) +
      ",\"max_wz\":" + jnum(cfg_.max_wz) + ",\"nav_gait\":" + std::to_string(cfg_.nav_gait) +
      ",\"cmd_sources\":" + jstr(srcs) + ",\"cmd_source\":" + jstr(source_) +
      ",\"exclusive_mode\":" + jstr(cfg_.exclusive_messages ? "messages" : "publishers") + "}");
  }

  rclcpp::Node::SharedPtr node() {return node_;}

  // Called once after DDS discovery settled. Creates the native publishers only
  // when motion is enabled and nobody else is sending (or, in publishers mode,
  // nobody else exists) on /NAV_CMD.
  void arm_publishers()
  {
    std::lock_guard<std::mutex> lock(m_);
    const double now = mono();
    const size_t others = node_->count_publishers("/NAV_CMD");
    const size_t foreign = foreign_recent(now, 3.0);
    if (!enable_) {
      event("dry_run", "{\"nav_cmd_publishers\":" + std::to_string(others) +
        ",\"foreign_nav_cmd_3s\":" + std::to_string(foreign) + "}");
      return;
    }
    if (cfg_.exclusive_messages ? foreign != 0 : others != 0) {
      fault_ = cfg_.exclusive_messages ?
        "another publisher is sending /NAV_CMD (" + std::to_string(foreign) + " msgs in 3 s); motion disabled" :
        "another /NAV_CMD publisher exists (" + std::to_string(others) + "); motion disabled";
      enable_ = false;
      event("fault", "{\"reason\":" + jstr(fault_) + "}");
      ROS_ERROR("%s", fault_.c_str());
      return;
    }
    rclcpp::QoS q(rclcpp::KeepLast(1));
    q.reliable();
    nav_pub_ = node_->create_publisher<drdds::msg::NavCmd>("/NAV_CMD", q);
    gait_pub_ = node_->create_publisher<drdds::msg::Gait>("/GAIT", q);
    state_cmd_pub_ = node_->create_publisher<drdds::msg::MotionState>("/MOTION_STATE", q);
    own_gid_ = nav_pub_->get_gid();
    have_gid_ = true;
    event("armed", "{\"nav_cmd_publishers_before\":" + std::to_string(others) + "}");
    ROS_WARN("MOTION ENABLED: /NAV_CMD, /MOTION_STATE, /GAIT publishers created");
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(m_);
    const double now = mono();
    maybe_start_gait_switch(now);
    step_sequence(now);
    if (now >= next_nav_) {
      // Fixed-rate schedule; resynchronise only after a stall.
      next_nav_ = (now - next_nav_ > 0.5) ? now + 1.0 / cfg_.nav_rate : next_nav_ + 1.0 / cfg_.nav_rate;
      nav_tick(now);
    }
    if (now >= next_exclusive_) {
      next_exclusive_ = now + 1.0;
      check_exclusive(now);
    }
    if (now >= next_gait_) {
      next_gait_ = now + 0.1;
      std_msgs::String g;
      g.data = gait_report(now);
      gait_pub1_.publish(g);
    }
    if (now >= next_state_) {
      next_state_ = now + 0.2;
      std_msgs::String s;
      s.data = state_json(now);
      state_pub_.publish(s);
    }
  }

  void shutdown()
  {
    std::lock_guard<std::mutex> lock(m_);
    if (nav_pub_) {
      for (int i = 0; i < 3; ++i) {send_nav(0, 0, 0);}
    }
    event("stop", "{}");
  }

private:
  // ------------------------------------------------------------ inputs
  void on_info(const drdds::msg::MotionInfo & m)
  {
    std::lock_guard<std::mutex> lock(m_);
    const double now = mono();
    const int64_t stamp = static_cast<int64_t>(m.header.stamp.sec) * 1000000000LL + m.header.stamp.nanosec;
    if (fb_have_ && fb_state_ != m.data.motion_state.state) {
      event("state_change", "{\"from\":" + std::to_string(fb_state_) + ",\"to\":" +
        std::to_string(m.data.motion_state.state) + "}");
    }
    if (fb_have_ && fb_gait_ != m.data.gait_state.gait) {
      event("gait_change", "{\"from\":" + std::to_string(fb_gait_) + ",\"to\":" +
        std::to_string(m.data.gait_state.gait) + "}");
    }
    fb_have_ = true;
    fb_rx_ = now;
    fb_state_ = m.data.motion_state.state;
    fb_gait_ = m.data.gait_state.gait;
    fb_vx_ = m.data.vel_x;
    fb_vy_ = m.data.vel_y;
    fb_wz_ = m.data.vel_yaw;
    fb_height_ = m.data.height;
    fb_stamp_ns_ = stamp;
  }

  void on_nav_seen(const rclcpp::MessageInfo & info)
  {
    std::lock_guard<std::mutex> lock(m_);
    const auto & gid = info.get_rmw_message_info().publisher_gid;
    if (have_gid_ && std::memcmp(gid.data, own_gid_.data, sizeof(gid.data)) == 0) {return;}
    ++foreign_total_;
    foreign_times_.push_back(mono());
    while (foreign_times_.size() > 200) {foreign_times_.pop_front();}
  }

  size_t foreign_recent(double now, double window) const
  {
    size_t n = 0;
    for (auto it = foreign_times_.rbegin(); it != foreign_times_.rend() && now - *it <= window; ++it) {++n;}
    return n;
  }

  void on_cmd(const geometry_msgs::Twist::ConstPtr & m, const std::string & source)
  {
    std::lock_guard<std::mutex> lock(m_);
    if (source != source_) {
      ++cmd_ignored_[source];
      return;
    }
    const double vals[3] = {m->linear.x, m->linear.y, m->angular.z};
    for (double v : vals) {
      if (!std::isfinite(v)) {
        ++cmd_rejected_;
        return;
      }
    }
    cmd_rx_ = mono();
    cmd_vx_ = m->linear.x;
    cmd_vy_ = m->linear.y;
    cmd_wz_ = m->angular.z;
    ++cmd_count_;
  }

  void on_gait_request(const std_msgs::String::ConstPtr & m)
  {
    std::lock_guard<std::mutex> lock(m_);
    const std::string g = trim(m->data);
    uint32_t code = g == "flat" ? g_flat_gait : g == "stairs" ? kNavStairs : g == "fast" ? kFast : g == "platform" ? kPlatform : 0;
    if (code == 0) {
      if (g != last_bad_gait_req_) {
        last_bad_gait_req_ = g;
        event("ignored", "{\"gait_request\":" + jstr(g) + ",\"reason\":\"not flat/fast/stairs/platform\"}");
      }
      return;
    }
    if (code != want_gait_) {event("gait_request", "{\"gait\":" + jstr(g) + "}");}
    want_gait_ = code;
    want_rx_ = mono();
  }

  void on_web(const std_msgs::String::ConstPtr & m)
  {
    std::lock_guard<std::mutex> lock(m_);
    const std::string c = trim(m->data);
    const double now = mono();
    event("web_cmd", "{\"data\":" + jstr(c) + "}");
    if (c == cfg_.w_stop) {
      latched_ = true;
      zero_until_ = now + cfg_.zero_hold;
      if (seq_ == Seq::StandUp || seq_ == Seq::StandSettle || seq_ == Seq::StandRl || seq_ == Seq::StandGait) {
        finish("stand_aborted", "Nav stop during stand sequence");
      }
    } else if (c == cfg_.w_continue) {
      if (!fault_.empty()) {
        event("rejected", "{\"command\":\"continue\",\"reason\":" + jstr(fault_) + "}");
      } else {
        latched_ = false;
      }
    } else if (c == cfg_.w_stand) {
      start_stand(now);
    } else if (c == cfg_.w_lie) {
      start_lie(now);
    } else if (c == cfg_.w_nav_mode) {
      start_nav_mode(now);
    } else if (c == cfg_.w_exit_nav) {
      // The developer guide documents only 0x3002/0x3003 for /GAIT, so there is no
      // documented way back to the manual gait from here: use the remote.
      event("ignored", "{\"data\":" + jstr(c) + ",\"reason\":\"exit navigation mode: use the remote (no documented /GAIT value)\"}");
    } else if (c.rfind(cfg_.w_source, 0) == 0 && c.size() > cfg_.w_source.size()) {
      select_source(trim(c.substr(cfg_.w_source.size())), now);
    } else {
      event("ignored", "{\"data\":" + jstr(c) + ",\"reason\":\"not an S10 command\"}");
    }
  }

  void select_source(const std::string & name, double now)
  {
    bool known = false;
    for (const auto & s : cfg_.cmd_sources) {known = known || s.first == name;}
    if (!known) {
      event("rejected", "{\"command\":\"source\",\"reason\":" + jstr("unknown source " + name) + "}");
      return;
    }
    if (name != source_) {
      event("source", "{\"from\":" + jstr(source_) + ",\"to\":" + jstr(name) + "}");
      source_ = name;
      cmd_rx_ = -1e9;      // never carry a stale command across sources
      zero_until_ = now + cfg_.zero_hold;
    }
  }

  // ------------------------------------------------------------ sequences
  bool fresh(double now) const {return fb_have_ && now - fb_rx_ < cfg_.info_timeout;}
  bool still(double limit) const
  {
    return std::fabs(fb_vx_) < limit && std::fabs(fb_vy_) < limit && std::fabs(fb_wz_) < limit;
  }

  bool reject(const char * what, const std::string & why)
  {
    event("rejected", std::string("{\"command\":\"") + what + "\",\"reason\":" + jstr(why) + "}");
    ROS_WARN("%s rejected: %s", what, why.c_str());
    return false;
  }

  bool start_stand(double now)
  {
    if (!fault_.empty()) {return reject("stand", fault_);}
    if (!fresh(now)) {return reject("stand", "no fresh /MOTION_INFO");}
    if (seq_ != Seq::None) {return reject("stand", std::string("busy: ") + seq_name(seq_));}
    if (fb_state_ != kIdle && fb_state_ != kBootDamping && fb_state_ != kLying) {
      return reject("stand", "robot state " + std::to_string(fb_state_) + " is not idle/boot-damping/lying");
    }
    if (!enable_) {
      event("dry_run_would_send", "{\"topic\":\"/MOTION_STATE\",\"state\":1}");
      return true;
    }
    send_state(kStanding);
    begin(Seq::StandUp, now, cfg_.stand_timeout);
    return true;
  }

  // "cmd1": return to the navigation gait while already in RL control (e.g. after the
  // operator changed gait with the remote). Standing up is "cmd4".
  bool start_nav_mode(double now)
  {
    if (!fault_.empty()) {return reject("nav_mode", fault_);}
    if (!fresh(now)) {return reject("nav_mode", "no fresh /MOTION_INFO");}
    if (seq_ != Seq::None) {return reject("nav_mode", std::string("busy: ") + seq_name(seq_));}
    if (fb_state_ != kRl) {return reject("nav_mode", "robot not in RL control (17); use cmd4 to stand first");}
    if (fb_gait_ == cfg_.nav_gait) {return true;}
    if (!enable_) {
      event("dry_run_would_send", "{\"topic\":\"/GAIT\",\"gait\":" + std::to_string(cfg_.nav_gait) + "}");
      return true;
    }
    send_gait(cfg_.nav_gait);
    begin(Seq::StandGait, now, cfg_.stand_timeout);
    return true;
  }

  bool start_lie(double now)
  {
    if (!fresh(now)) {return reject("lie_down", "no fresh /MOTION_INFO");}
    if (seq_ == Seq::LieStill || seq_ == Seq::LieDown) {return reject("lie_down", "already lying down");}
    if (fb_state_ == kLying) {return reject("lie_down", "already lying");}
    if (seq_ != Seq::None) {finish("stand_aborted", "lie down requested");}
    lie_block_ = true;           // forces zero velocity from now on
    zero_until_ = now + cfg_.zero_hold;
    if (!enable_) {
      event("dry_run_would_send", "{\"topic\":\"/MOTION_STATE\",\"state\":4,\"after\":\"robot still\"}");
      lie_block_ = false;
      return true;
    }
    still_since_ = -1;
    begin(Seq::LieStill, now, cfg_.still_timeout);
    return true;
  }

  // A fresh gait request that differs from the robot's gait starts a switch when nothing
  // else is going on: zero velocity -> measured still -> /GAIT -> confirmed.
  void maybe_start_gait_switch(double now)
  {
    if (want_gait_ == 0 || now - want_rx_ > 1.0 || seq_ != Seq::None || !fault_.empty()) {return;}
    if (!fresh(now) || fb_state_ != kRl || fb_gait_ == want_gait_) {return;}
    if (!enable_) {
      if (dry_gait_logged_ != want_gait_) {
        dry_gait_logged_ = want_gait_;
        event("dry_run_would_send", "{\"topic\":\"/GAIT\",\"gait\":" + std::to_string(want_gait_) +
          ",\"after\":\"robot still\"}");
      }
      return;
    }
    gait_target_ = want_gait_;
    gait_block_ = true;
    zero_until_ = now + cfg_.zero_hold;
    still_since_ = -1;
    begin(Seq::GaitStill, now, cfg_.gait_still_timeout);
  }

  void begin(Seq s, double now, double timeout)
  {
    seq_ = s;
    seq_deadline_ = now + timeout;
    seq_last_send_ = now;
    seq_sends_ = 1;
    event("sequence", std::string("{\"step\":\"") + seq_name(s) + "\"}");
  }

  void finish(const char * result, const std::string & detail)
  {
    event(result, "{\"step\":\"" + std::string(seq_name(seq_)) + "\",\"detail\":" + jstr(detail) + "}");
    seq_ = Seq::None;
    lie_block_ = false;
    gait_block_ = false;
  }

  void resend_or_timeout(double now, const std::function<void()> & send, const char * what)
  {
    if (now > seq_deadline_) {
      finish("sequence_failed", std::string(what) + " not confirmed before timeout");
      latched_ = true;   // stay safe: no velocity until "Nav continue"
      zero_until_ = now + cfg_.zero_hold;
    } else if (now - seq_last_send_ > cfg_.resend && seq_sends_ < 3) {
      send();
      seq_last_send_ = now;
      ++seq_sends_;
    }
  }

  void step_sequence(double now)
  {
    if (seq_ == Seq::None) {return;}
    if (!fresh(now)) {
      finish("sequence_failed", "lost /MOTION_INFO");
      latched_ = true;
      return;
    }
    switch (seq_) {
      case Seq::StandUp:
        if (fb_state_ == kStanding) {
          seq_ = Seq::StandSettle;
          settle_since_ = now;
          event("sequence", "{\"step\":\"stand:settle\"}");
        } else if (fb_state_ == kRl) {
          send_gait(cfg_.nav_gait);
          begin(Seq::StandGait, now, cfg_.stand_timeout);
        } else {
          resend_or_timeout(now, [this] {send_state(kStanding);}, "standing (state 1)");
        }
        break;
      case Seq::StandSettle:
        if (fb_state_ != kStanding) {
          finish("sequence_failed", "left standing state while settling: " + std::to_string(fb_state_));
          latched_ = true;
        } else if (now - settle_since_ >= cfg_.stand_settle) {
          send_state(kRl);
          begin(Seq::StandRl, now, cfg_.stand_timeout);
        }
        break;
      case Seq::StandRl:
        if (fb_state_ == kRl) {
          send_gait(cfg_.nav_gait);
          begin(Seq::StandGait, now, cfg_.stand_timeout);
        } else {
          resend_or_timeout(now, [this] {send_state(kRl);}, "RL control (state 17)");
        }
        break;
      case Seq::StandGait:
        if (fb_gait_ == cfg_.nav_gait && fb_state_ == kRl) {
          finish("stand_complete", "RL control with navigation gait");
        } else {
          resend_or_timeout(now, [this] {send_gait(cfg_.nav_gait);}, "navigation gait");
        }
        break;
      case Seq::LieStill:
        if (still(cfg_.still_speed)) {
          if (still_since_ < 0) {still_since_ = now;}
          if (now - still_since_ >= cfg_.still_time) {
            send_state(kLying);
            seq_ = Seq::LieDown;
            seq_deadline_ = now + cfg_.lie_timeout;
            seq_last_send_ = now;
            seq_sends_ = 1;
            event("sequence", "{\"step\":\"lie:wait_lying\"}");
          }
        } else {
          still_since_ = -1;
          if (now > seq_deadline_) {
            finish("sequence_failed", "robot did not become still; not lying down");
            latched_ = true;
          }
        }
        break;
      case Seq::LieDown:
        if (fb_state_ == kLying) {
          finish("lie_complete", "state 4");
        } else {
          resend_or_timeout(now, [this] {send_state(kLying);}, "lying (state 4)");
        }
        break;
      case Seq::GaitStill:
        if (fb_state_ != kRl) {
          finish("sequence_failed", "left RL control during gait switch: " + std::to_string(fb_state_));
          latched_ = true;
        } else if (still(cfg_.gait_still_speed)) {
          if (still_since_ < 0) {still_since_ = now;}
          if (now - still_since_ >= cfg_.gait_still_time) {
            send_gait(gait_target_);
            seq_ = Seq::GaitWait;
            seq_deadline_ = now + cfg_.gait_timeout;
            seq_last_send_ = now;
            seq_sends_ = 1;
            event("sequence", "{\"step\":\"gait:wait_confirm\"}");
          }
        } else {
          still_since_ = -1;
          if (now > seq_deadline_) {
            finish("sequence_failed", "robot did not become still; gait not switched");
            latched_ = true;
          }
        }
        break;
      case Seq::GaitWait:
        if (fb_gait_ == gait_target_ && fb_state_ == kRl) {
          finish("gait_switched", std::string("{\"gait\":\"") + gait_name(gait_target_) + "\"}");
        } else {
          resend_or_timeout(now, [this] {send_gait(gait_target_);}, "gait switch");
        }
        break;
      case Seq::None:
        break;
    }
  }

  // ------------------------------------------------------------ velocity
  void nav_tick(double now)
  {
    const bool nav_mode = fresh(now) && fb_state_ == kRl && (fb_gait_ == kNavFlat || fb_gait_ == kNavStairs || fb_gait_ == g_flat_gait || fb_gait_ == kFast || fb_gait_ == kPlatform);
    const bool cmd_fresh = now - cmd_rx_ < cfg_.cmd_timeout;
    const bool allowed = fault_.empty() && !latched_ && !lie_block_ && !gait_block_ && seq_ == Seq::None && nav_mode;
    if (allowed && cmd_fresh) {
      const double vx = std::clamp(cmd_vx_, -cfg_.max_vx, cfg_.max_vx);
      const double vy = std::clamp(cmd_vy_, -cfg_.max_vy, cfg_.max_vy);
      const double wz = std::clamp(cmd_wz_, -cfg_.max_wz, cfg_.max_wz);
      if (vx != cmd_vx_ || vy != cmd_vy_ || wz != cmd_wz_) {++clamped_;}
      if (!moving_) {
        moving_ = true;
        event("velocity_start", "{\"source\":" + jstr(source_) + "}");
      }
      last_out_[0] = vx; last_out_[1] = vy; last_out_[2] = wz;
      send_nav(vx, vy, wz);
      zero_until_ = now + cfg_.zero_hold;
    } else {
      if (moving_) {
        moving_ = false;
        event("velocity_stop", std::string("{\"reason\":\"") +
          (!fault_.empty() ? "fault" : latched_ ? "latched" : lie_block_ ? "lie_down" :
          gait_block_ ? "gait_switch" : seq_ != Seq::None ? "sequence" :
          !nav_mode ? "not_navigation_mode" : "cmd_vel_timeout") + "\"}");
      }
      last_out_[0] = last_out_[1] = last_out_[2] = 0;
      if (now < zero_until_) {send_nav(0, 0, 0);}
    }
  }

  void check_exclusive(double now)
  {
    nav_publishers_ = node_->count_publishers("/NAV_CMD");
    foreign_1s_ = foreign_recent(now, 1.0);
    if (!nav_pub_ || !fault_.empty()) {return;}
    if (cfg_.exclusive_messages ? foreign_1s_ != 0 : nav_publishers_ > 1) {
      fault_ = cfg_.exclusive_messages ?
        "another publisher sent /NAV_CMD (" + std::to_string(foreign_1s_) + " msgs in 1 s); restart required" :
        "second /NAV_CMD publisher appeared (" + std::to_string(nav_publishers_) + "); restart required";
      zero_until_ = now + cfg_.zero_hold;
      event("fault", "{\"reason\":" + jstr(fault_) + "}");
      ROS_ERROR("%s", fault_.c_str());
    }
  }

  // ------------------------------------------------------------ outputs
  builtin_interfaces::msg::Time command_stamp()
  {
    int64_t ns = wall_ns();
    if (cfg_.stamp_robot && fb_have_) {
      ns = fb_stamp_ns_ + static_cast<int64_t>((mono() - fb_rx_) * 1e9);
    }
    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
  }

  void send_nav(double vx, double vy, double wz)
  {
    ++nav_sent_;
    if (!nav_pub_) {return;}
    drdds::msg::NavCmd m;
    m.header.stamp = command_stamp();
    m.data.x_vel = static_cast<float>(vx);
    m.data.y_vel = static_cast<float>(vy);
    m.data.yaw_vel = static_cast<float>(wz);
    nav_pub_->publish(m);
  }

  void send_state(int32_t state)
  {
    event("send", "{\"topic\":\"/MOTION_STATE\",\"state\":" + std::to_string(state) + "}");
    if (!state_cmd_pub_) {return;}
    drdds::msg::MotionState m;
    m.header.stamp = command_stamp();
    m.data.state = state;
    state_cmd_pub_->publish(m);
  }

  void send_gait(uint32_t gait)
  {
    event("send", "{\"topic\":\"/GAIT\",\"gait\":" + std::to_string(gait) + "}");
    if (!gait_pub_) {return;}
    drdds::msg::Gait m;
    m.header.stamp = command_stamp();
    m.data.gait = gait;
    gait_pub_->publish(m);
  }

  void event(const std::string & name, const std::string & json)
  {
    last_event_ = name;
    std::ostringstream o;
    o << "{\"wall_ns\":" << wall_ns() << ",\"event\":" << jstr(name) << ",\"detail\":" << json << "}";
    if (events_) {events_ << o.str() << "\n" << std::flush;}
    ROS_INFO("event %s %s", name.c_str(), json.c_str());
  }

  std::string gait_report(double now) const
  {
    if (seq_ == Seq::GaitStill || seq_ == Seq::GaitWait) {return "switching";}
    if (!fresh(now) || fb_state_ != kRl) {return "none";}
    return gait_name(fb_gait_);
  }

  std::string state_json(double now)
  {
    std::ostringstream o;
    o << "{\"enable_motion\":" << (enable_ ? "true" : "false")
      << ",\"publishers_created\":" << (nav_pub_ ? "true" : "false")
      << ",\"fault\":" << jstr(fault_)
      << ",\"latched_stop\":" << (latched_ ? "true" : "false")
      << ",\"sequence\":" << jstr(seq_name(seq_))
      << ",\"moving\":" << (moving_ ? "true" : "false")
      << ",\"cmd_source\":" << jstr(source_)
      << ",\"gait\":" << jstr(gait_report(now))
      << ",\"gait_request\":" << jstr(want_gait_ && now - want_rx_ < 1.0 ? gait_name(want_gait_) : "")
      << ",\"feedback\":{\"fresh\":" << (fresh(now) ? "true" : "false")
      << ",\"age_s\":" << jnum(fb_have_ ? now - fb_rx_ : NAN)
      << ",\"state\":" << fb_state_ << ",\"gait\":" << fb_gait_
      << ",\"vel\":[" << jnum(fb_vx_) << "," << jnum(fb_vy_) << "," << jnum(fb_wz_) << "]"
      << ",\"height\":" << jnum(fb_height_) << "}"
      << ",\"cmd_vel\":{\"age_s\":" << jnum(cmd_count_ ? now - cmd_rx_ : NAN)
      << ",\"in\":[" << jnum(cmd_vx_) << "," << jnum(cmd_vy_) << "," << jnum(cmd_wz_) << "]"
      << ",\"out\":[" << jnum(last_out_[0]) << "," << jnum(last_out_[1]) << "," << jnum(last_out_[2]) << "]"
      << ",\"received\":" << cmd_count_ << ",\"rejected\":" << cmd_rejected_ << ",\"clamped\":" << clamped_
      << ",\"ignored\":{";
    bool first = true;
    for (const auto & kv : cmd_ignored_) {
      o << (first ? "" : ",") << jstr(kv.first) << ":" << kv.second;
      first = false;
    }
    o << "}}"
      << ",\"nav_cmd_sent\":" << nav_sent_
      << ",\"nav_cmd_publishers\":" << nav_publishers_
      << ",\"nav_cmd_foreign_1s\":" << foreign_1s_
      << ",\"nav_cmd_foreign_total\":" << foreign_total_
      << ",\"limits\":[" << jnum(cfg_.max_vx) << "," << jnum(cfg_.max_vy) << "," << jnum(cfg_.max_wz) << "]"
      << ",\"last_event\":" << jstr(last_event_) << "}";
    return o.str();
  }

  Config cfg_;
  bool enable_;
  std::mutex m_;
  std::ofstream events_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<drdds::msg::MotionInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<drdds::msg::NavCmd>::SharedPtr nav_mon_sub_;
  rclcpp::Publisher<drdds::msg::NavCmd>::SharedPtr nav_pub_;
  rclcpp::Publisher<drdds::msg::Gait>::SharedPtr gait_pub_;
  rclcpp::Publisher<drdds::msg::MotionState>::SharedPtr state_cmd_pub_;
  std::vector<ros::Subscriber> cmd_subs_;
  ros::Subscriber web_sub_, gait_req_sub_;
  ros::Publisher state_pub_, gait_pub1_;

  rmw_gid_t own_gid_{};
  bool have_gid_ = false;
  std::deque<double> foreign_times_;
  uint64_t foreign_total_ = 0;
  size_t foreign_1s_ = 0;

  bool fb_have_ = false;
  double fb_rx_ = 0;
  int32_t fb_state_ = -1;
  uint32_t fb_gait_ = 0;
  double fb_vx_ = 0, fb_vy_ = 0, fb_wz_ = 0, fb_height_ = NAN;
  int64_t fb_stamp_ns_ = 0;

  double cmd_rx_ = -1e9, cmd_vx_ = 0, cmd_vy_ = 0, cmd_wz_ = 0;
  uint64_t cmd_count_ = 0, cmd_rejected_ = 0, clamped_ = 0, nav_sent_ = 0;
  std::map<std::string, uint64_t> cmd_ignored_;
  size_t nav_publishers_ = 0;
  double last_out_[3] = {0, 0, 0};

  bool latched_;
  bool lie_block_ = false;
  bool gait_block_ = false;
  bool moving_ = false;
  std::string fault_;
  std::string last_event_;
  std::string source_;
  double zero_until_ = 0;
  Seq seq_ = Seq::None;
  double seq_deadline_ = 0, seq_last_send_ = 0, settle_since_ = 0, still_since_ = -1;
  int seq_sends_ = 0;
  uint32_t want_gait_ = 0, gait_target_ = 0, dry_gait_logged_ = 0;
  double want_rx_ = -1e9;
  std::string last_bad_gait_req_;
  double next_nav_ = 0, next_exclusive_ = 0, next_state_ = 0, next_gait_ = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path, event_log;
  bool enable_motion = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (a == "--event-log" && i + 1 < argc) {
      event_log = argv[++i];
    } else if (a == "--enable-motion") {
      enable_motion = true;
    } else {
      std::cerr << "usage: s10_ros1_control --config control.yaml [--event-log file.jsonl] [--enable-motion]\n";
      return 2;
    }
  }
  if (config_path.empty()) {
    std::cerr << "--config is required\n";
    return 2;
  }
  Config cfg;
  try {
    cfg = load(config_path);
  } catch (const std::exception & e) {
    std::cerr << "configuration error: " << e.what() << std::endl;
    return 2;
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  int ros1_argc = 1;
  char * ros1_argv[] = {argv[0], nullptr};
  ros::init(ros1_argc, ros1_argv, "s10_ros1_control", ros::init_options::NoSigintHandler);
  while (!g_stop && !ros::master::check()) {
    ROS_WARN_THROTTLE(5, "waiting for ROS 1 master at %s", ros::master::getURI().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (g_stop) {return 0;}
  rclcpp::init(1, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  auto controller = std::make_unique<Controller>(cfg, enable_motion, event_log);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller->node());
  std::thread ros2_thread([&executor]() {executor.spin();});
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Let DDS discovery settle (and foreign /NAV_CMD traffic show up) before arming.
  const double discovery_end = mono() + 3.0;
  while (!g_stop && mono() < discovery_end) {std::this_thread::sleep_for(std::chrono::milliseconds(50));}
  if (!g_stop) {controller->arm_publishers();}

  while (!g_stop && ros::ok() && rclcpp::ok()) {
    controller->tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  controller->shutdown();
  spinner.stop();
  executor.cancel();
  if (ros2_thread.joinable()) {ros2_thread.join();}
  controller.reset();
  rclcpp::shutdown();
  ros::shutdown();
  return 0;
}
