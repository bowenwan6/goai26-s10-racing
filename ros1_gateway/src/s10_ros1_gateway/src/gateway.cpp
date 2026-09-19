// s10_ros1_gateway: one-way ROS 2 -> ROS 1 gateway for the S10 sensor topics.
//
// Inputs (per topic, from topics.yaml)
//   source: dds  rclcpp generic (serialized) subscription on this host.
//   source: tap  raw CDR frames pushed over TCP by tap/s10_lidar_tap.py on 106,
//                because 106 keeps /LIDAR/POINTS host-local.
// Conversion
//   ros1_bridge FactoryInterface::convert_2_to_1_generic, i.e. upstream's
//   generated per-type code. Stamps, frame ids, covariances and the
//   PointCloud2 field layout pass through unchanged. ROS 1 Header.seq is 0
//   because ROS 2 has no seq.
// Output
//   One roscpp publisher per topic (topic_tools::ShapeShifter carrying the ROS 1
//   md5sum/definition reported by the factory).
// Safety
//   The process never creates a ROS 2 publisher, service or parameter server,
//   and ignores ROS remapping arguments: topic names come only from the YAML.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "topic_tools/shape_shifter.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "ros1_bridge/bridge.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

constexpr char kVersion[] = "s10_ros1_gateway 1.0.0";
std::atomic<bool> g_stop{false};

void on_signal(int) {g_stop = true;}

double steady_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

int64_t wall_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string json_str(const std::string & s)
{
  std::ostringstream out;
  out << '"';
  for (unsigned char c : s) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        } else {
          out << c;
        }
    }
  }
  out << '"';
  return out.str();
}

std::string json_num(double v)
{
  if (!std::isfinite(v)) {return "null";}
  std::ostringstream out;
  out << std::setprecision(9) << v;
  return out.str();
}

std::string hex_bytes(const uint8_t * data, size_t n)
{
  std::ostringstream out;
  for (size_t i = 0; i < n; ++i) {
    out << std::hex << std::setw(2) << std::setfill('0') << int(data[i]);
  }
  return out.str();
}

// ---------------------------------------------------------------- config

struct TopicConfig
{
  std::string ros2_topic;
  std::string ros1_topic;
  std::string ros1_type;   // e.g. sensor_msgs/PointCloud2
  std::string ros2_type;   // e.g. sensor_msgs/msg/PointCloud2
  std::string source;      // dds | tap
  int ros1_queue_size = 10;
  int ros2_depth = 10;
  std::string ros2_reliability = "best_effort";
};

struct TapConfig
{
  bool enabled = true;
  std::string listen_address = "10.21.33.102";
  int listen_port = 47631;
  std::set<std::string> allowed_peers{"10.21.33.106"};
  uint64_t max_payload_bytes = 64ull << 20;
  double idle_timeout_s = 5.0;
};

struct GatewayConfig
{
  std::string ros1_node_name = "s10_ros1_gateway";
  std::string ros2_node_name = "s10_ros1_gateway";
  std::string status_topic = "/s10_ros1_gateway/status";
  std::string status_file;
  double status_period_s = 1.0;
  TapConfig tap;
  std::vector<TopicConfig> topics;
};

std::string resolve_relative(const std::string & path, const std::string & base_file)
{
  if (path.empty() || path[0] == '/') {return path;}
  auto slash = base_file.find_last_of('/');
  return slash == std::string::npos ? path : base_file.substr(0, slash + 1) + path;
}

GatewayConfig load_config(const std::string & config_path, const std::string & topics_path)
{
  GatewayConfig cfg;
  YAML::Node root = YAML::LoadFile(config_path);
  if (root["ros1_node_name"]) {cfg.ros1_node_name = root["ros1_node_name"].as<std::string>();}
  if (root["ros2_node_name"]) {cfg.ros2_node_name = root["ros2_node_name"].as<std::string>();}
  if (root["status_topic"]) {cfg.status_topic = root["status_topic"].as<std::string>();}
  if (root["status_file"]) {
    cfg.status_file = resolve_relative(root["status_file"].as<std::string>(), config_path);
  }
  if (root["status_period_s"]) {cfg.status_period_s = root["status_period_s"].as<double>();}
  if (YAML::Node tap = root["tap"]) {
    if (tap["enabled"]) {cfg.tap.enabled = tap["enabled"].as<bool>();}
    if (tap["listen_address"]) {cfg.tap.listen_address = tap["listen_address"].as<std::string>();}
    if (tap["listen_port"]) {cfg.tap.listen_port = tap["listen_port"].as<int>();}
    if (tap["allowed_peers"]) {
      cfg.tap.allowed_peers.clear();
      for (const auto & p : tap["allowed_peers"]) {cfg.tap.allowed_peers.insert(p.as<std::string>());}
    }
    if (tap["max_payload_bytes"]) {cfg.tap.max_payload_bytes = tap["max_payload_bytes"].as<uint64_t>();}
    if (tap["idle_timeout_s"]) {cfg.tap.idle_timeout_s = tap["idle_timeout_s"].as<double>();}
  }
  if (!(cfg.status_period_s >= 0.1 && cfg.status_period_s <= 60)) {
    throw std::runtime_error("status_period_s must be within [0.1, 60]");
  }

  YAML::Node topics = YAML::LoadFile(topics_path)["topics"];
  if (!topics || !topics.IsSequence() || topics.size() == 0) {
    throw std::runtime_error(topics_path + ": no topics list");
  }
  std::set<std::string> ros1_names;
  std::set<std::string> ros2_names;
  for (const auto & t : topics) {
    TopicConfig tc;
    tc.ros2_topic = t["ros2_topic"].as<std::string>();
    tc.ros1_topic = t["ros1_topic"].as<std::string>();
    tc.ros1_type = t["type"].as<std::string>();
    tc.source = t["source"] ? t["source"].as<std::string>() : "dds";
    if (t["ros1_queue_size"]) {tc.ros1_queue_size = t["ros1_queue_size"].as<int>();}
    if (t["ros2_depth"]) {tc.ros2_depth = t["ros2_depth"].as<int>();}
    if (t["ros2_reliability"]) {tc.ros2_reliability = t["ros2_reliability"].as<std::string>();}
    auto slash = tc.ros1_type.find('/');
    if (slash == std::string::npos || tc.ros1_type.find('/', slash + 1) != std::string::npos) {
      throw std::runtime_error("type must look like pkg/Msg: " + tc.ros1_type);
    }
    tc.ros2_type = tc.ros1_type.substr(0, slash) + "/msg/" + tc.ros1_type.substr(slash + 1);
    if (tc.ros1_topic.empty() || tc.ros1_topic[0] != '/' || tc.ros2_topic.empty() ||
      tc.ros2_topic[0] != '/')
    {
      throw std::runtime_error("topic names must be absolute: " + tc.ros2_topic + " -> " + tc.ros1_topic);
    }
    if (tc.source != "dds" && tc.source != "tap") {
      throw std::runtime_error("source must be dds or tap: " + tc.source);
    }
    if (tc.ros2_reliability != "best_effort" && tc.ros2_reliability != "reliable") {
      throw std::runtime_error("ros2_reliability must be best_effort or reliable");
    }
    if (tc.ros1_queue_size < 1 || tc.ros1_queue_size > 10000 || tc.ros2_depth < 1 || tc.ros2_depth > 10000) {
      throw std::runtime_error("queue sizes must be within [1, 10000]");
    }
    if (!ros1_names.insert(tc.ros1_topic).second || !ros2_names.insert(tc.ros2_topic).second) {
      throw std::runtime_error("duplicate topic: " + tc.ros2_topic + " -> " + tc.ros1_topic);
    }
    cfg.topics.push_back(tc);
  }
  return cfg;
}

// ---------------------------------------------------------------- routes

struct Meta
{
  int64_t recv_ns = 0;   // reception time at the receiving side (102 DDS / 106 tap)
  int64_t src_ns = 0;    // DDS source timestamp
  int64_t pub_seq = -1;  // DDS publication sequence number
  int64_t tap_dropped = -1;
  std::string gid;       // publisher GID (dds only)
};

class Route
{
public:
  Route(const TopicConfig & cfg, ros::NodeHandle & nh)
  : cfg_(cfg)
  {
    factory_ = ros1_bridge::get_factory(cfg.ros1_type, cfg.ros2_type);
    if (!factory_) {
      throw std::runtime_error("ros1_bridge has no factory for " + cfg.ros2_type);
    }
    md5_ = factory_->get_ros1_md5sum();
    datatype_ = factory_->get_ros1_data_type();
    definition_ = factory_->get_ros1_message_definition();
    topic_tools::ShapeShifter prototype;
    prototype.morph(md5_, datatype_, definition_, "0");
    pub_ = prototype.advertise(nh, cfg.ros1_topic, cfg.ros1_queue_size, false);
  }

  const TopicConfig & cfg() const {return cfg_;}

  void forward(const rclcpp::SerializedMessage & cdr, const Meta & meta)
  {
    const double t_rx = steady_s();
    const int64_t wall_rx = wall_ns();
    std::vector<uint8_t> ros1;
    const bool ok = factory_->convert_2_to_1_generic(cdr, ros1);
    int64_t stamp_ns = -1;
    std::string frame_id;
    const bool header_ok = ok && parse_header(ros1, stamp_ns, frame_id);
    if (ok) {
      topic_tools::ShapeShifter shape;
      shape.morph(md5_, datatype_, definition_, "0");
      ros::serialization::IStream stream(ros1.data(), static_cast<uint32_t>(ros1.size()));
      shape.read(stream);
      pub_.publish(shape);
    }
    const double convert_ms = (steady_s() - t_rx) * 1e3;

    std::lock_guard<std::mutex> lock(mutex_);
    ++received_;
    bytes_in_ += cdr.size();
    if (!ok || !header_ok) {
      ++convert_errors_;
      return;
    }
    ++published_;
    bytes_out_ += ros1.size();
    convert_ms_max_ = std::max(convert_ms_max_, convert_ms);
    convert_ms_sum_ += convert_ms;
    recent_.push_back(t_rx);
    while (!recent_.empty() && recent_.front() < t_rx - kWindowS) {recent_.pop_front();}
    last_rx_steady_ = t_rx;
    if (last_stamp_ns_ >= 0) {
      if (stamp_ns < last_stamp_ns_) {++stamp_backwards_;}
      if (stamp_ns == last_stamp_ns_) {++stamp_repeats_;}
    }
    last_stamp_ns_ = stamp_ns;
    last_frame_id_ = frame_id;
    last_stamp_minus_wall_s_ = (stamp_ns - wall_rx) / 1e9;
    stamp_minus_wall_.push_back({t_rx, last_stamp_minus_wall_s_});
    while (!stamp_minus_wall_.empty() && stamp_minus_wall_.front().first < t_rx - kWindowS) {
      stamp_minus_wall_.pop_front();
    }
    if (!meta.gid.empty()) {++publishers_[meta.gid];}
    if (meta.pub_seq >= 0) {
      if (last_pub_seq_ >= 0) {
        if (meta.pub_seq <= last_pub_seq_) {
          ++pub_seq_backwards_;
        } else if (meta.pub_seq > last_pub_seq_ + 1) {
          pub_seq_gaps_ += static_cast<uint64_t>(meta.pub_seq - last_pub_seq_ - 1);
        }
      }
      last_pub_seq_ = meta.pub_seq;
    }
    if (meta.tap_dropped >= 0) {tap_dropped_ = meta.tap_dropped;}
    if (meta.recv_ns > 0 && meta.src_ns > 0) {last_transport_s_ = (meta.recv_ns - meta.src_ns) / 1e9;}
  }

  std::string status_json()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = steady_s();
    while (!recent_.empty() && recent_.front() < now - kWindowS) {recent_.pop_front();}
    double rate = NAN;
    if (recent_.size() >= 2) {
      rate = (recent_.size() - 1) / (recent_.back() - recent_.front());
    }
    double lag_min = NAN, lag_max = NAN;
    for (const auto & s : stamp_minus_wall_) {
      if (s.first < now - kWindowS) {continue;}
      lag_min = std::isnan(lag_min) ? s.second : std::min(lag_min, s.second);
      lag_max = std::isnan(lag_max) ? s.second : std::max(lag_max, s.second);
    }
    std::ostringstream o;
    o << "{\"ros2_topic\":" << json_str(cfg_.ros2_topic)
      << ",\"ros1_topic\":" << json_str(cfg_.ros1_topic)
      << ",\"type\":" << json_str(cfg_.ros1_type)
      << ",\"ros1_md5\":" << json_str(md5_)
      << ",\"source\":" << json_str(cfg_.source)
      << ",\"received\":" << received_
      << ",\"published\":" << published_
      << ",\"convert_errors\":" << convert_errors_
      << ",\"bytes_in\":" << bytes_in_
      << ",\"bytes_out\":" << bytes_out_
      << ",\"rate_hz\":" << json_num(rate)
      << ",\"last_rx_age_s\":" << json_num(last_rx_steady_ < 0 ? NAN : now - last_rx_steady_)
      << ",\"last_stamp_ns\":" << last_stamp_ns_
      << ",\"last_frame_id\":" << json_str(last_frame_id_)
      << ",\"stamp_minus_local_clock_s\":{\"last\":" << json_num(last_stamp_minus_wall_s_)
      << ",\"min\":" << json_num(lag_min) << ",\"max\":" << json_num(lag_max) << "}"
      << ",\"stamp_backwards\":" << stamp_backwards_
      << ",\"stamp_repeats\":" << stamp_repeats_
      << ",\"pub_seq_gaps\":" << pub_seq_gaps_
      << ",\"pub_seq_backwards\":" << pub_seq_backwards_
      << ",\"tap_reported_dropped\":" << tap_dropped_
      << ",\"transport_last_s\":" << json_num(last_transport_s_)
      << ",\"convert_ms_avg\":" << json_num(published_ ? convert_ms_sum_ / published_ : NAN)
      << ",\"convert_ms_max\":" << json_num(convert_ms_max_)
      << ",\"ros1_subscribers\":" << pub_.getNumSubscribers()
      << ",\"publishers\":{";
    bool first = true;
    for (const auto & p : publishers_) {
      o << (first ? "" : ",") << json_str(p.first) << ":" << p.second;
      first = false;
    }
    o << "}}";
    return o.str();
  }

  std::string summary_line()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = steady_s();
    while (!recent_.empty() && recent_.front() < now - kWindowS) {recent_.pop_front();}
    double rate = recent_.size() >= 2 ? (recent_.size() - 1) / (recent_.back() - recent_.front()) : 0.0;
    std::ostringstream o;
    o << cfg_.ros2_topic << " -> " << cfg_.ros1_topic << ": " << std::fixed << std::setprecision(1)
      << rate << " Hz, published " << published_ << ", errors " << convert_errors_
      << ", stamp_backwards " << stamp_backwards_ << ", ros1_subscribers " << pub_.getNumSubscribers();
    return o.str();
  }

private:
  static constexpr double kWindowS = 5.0;

  // All three supported types start with std_msgs/Header:
  // uint32 seq, uint32 sec, uint32 nsec, uint32 len + frame_id bytes.
  static bool parse_header(const std::vector<uint8_t> & b, int64_t & stamp_ns, std::string & frame)
  {
    if (b.size() < 16) {return false;}
    uint32_t sec = 0, nsec = 0, len = 0;
    std::memcpy(&sec, b.data() + 4, 4);
    std::memcpy(&nsec, b.data() + 8, 4);
    std::memcpy(&len, b.data() + 12, 4);
    if (16ull + len > b.size() || nsec >= 1000000000u) {return false;}
    frame.assign(reinterpret_cast<const char *>(b.data() + 16), len);
    stamp_ns = static_cast<int64_t>(sec) * 1000000000LL + nsec;
    return true;
  }

  TopicConfig cfg_;
  std::shared_ptr<ros1_bridge::FactoryInterface> factory_;
  std::string md5_, datatype_, definition_;
  ros::Publisher pub_;

  std::mutex mutex_;
  uint64_t received_ = 0, published_ = 0, convert_errors_ = 0, bytes_in_ = 0, bytes_out_ = 0;
  uint64_t stamp_backwards_ = 0, stamp_repeats_ = 0, pub_seq_gaps_ = 0, pub_seq_backwards_ = 0;
  int64_t last_stamp_ns_ = -1, last_pub_seq_ = -1, tap_dropped_ = -1;
  std::string last_frame_id_;
  double last_rx_steady_ = -1, last_stamp_minus_wall_s_ = NAN, last_transport_s_ = NAN;
  double convert_ms_max_ = 0, convert_ms_sum_ = 0;
  std::deque<double> recent_;
  std::deque<std::pair<double, double>> stamp_minus_wall_;
  std::map<std::string, uint64_t> publishers_;
};

// ---------------------------------------------------------------- tap server
//
// Frame = 16-byte little-endian prefix + JSON header + payload.
//   prefix: "S10L" | u16 version=1 | u16 kind | u32 header_len | u32 payload_len
//   kind 1 HELLO     tap -> gw  {"tap":..,"host":..,"pid":..,"topics":[{"topic":..,"type":..}]}
//   kind 2 MESSAGE   tap -> gw  {"topic":..,"type":..,"recv_ns":..,"src_ns":..,"pub_seq":..,
//                                "tap_seq":..,"tap_dropped":..} + CDR payload
//   kind 3 HEARTBEAT tap -> gw  {"received":..,"sent":..,"dropped":..}
//   kind 4 WELCOME   gw -> tap  {"accepted":[..],"rejected":[..],"gateway":..}

enum FrameKind : uint16_t {kHello = 1, kMessage = 2, kHeartbeat = 3, kWelcome = 4};

class TapServer
{
public:
  TapServer(TapConfig cfg, std::map<std::string, std::shared_ptr<Route>> routes)
  : cfg_(std::move(cfg)), routes_(std::move(routes)) {}

  void run()
  {
    int ls = ::socket(AF_INET, SOCK_STREAM, 0);
    if (ls < 0) {throw std::runtime_error("tap: socket() failed");}
    int one = 1;
    ::setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(cfg_.listen_port));
    if (::inet_pton(AF_INET, cfg_.listen_address.c_str(), &addr.sin_addr) != 1) {
      ::close(ls);
      throw std::runtime_error("tap: bad listen_address " + cfg_.listen_address);
    }
    while (!g_stop && ::bind(ls, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      // end0 may come up after us (cable, NetworkManager); keep retrying.
      set_state("bind_failed: " + std::string(std::strerror(errno)));
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (g_stop) {::close(ls); return;}
    ::listen(ls, 2);
    set_state("listening");
    ROS_INFO("tap: listening on %s:%d, allowed peers: %zu", cfg_.listen_address.c_str(),
      cfg_.listen_port, cfg_.allowed_peers.size());
    while (!g_stop) {
      pollfd pfd{ls, POLLIN, 0};
      if (::poll(&pfd, 1, 200) <= 0) {continue;}
      sockaddr_in peer{};
      socklen_t plen = sizeof(peer);
      int fd = ::accept(ls, reinterpret_cast<sockaddr *>(&peer), &plen);
      if (fd < 0) {continue;}
      char ip[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
      if (!cfg_.allowed_peers.count(ip)) {
        ROS_WARN("tap: rejected connection from %s (not in allowed_peers)", ip);
        std::lock_guard<std::mutex> lock(mutex_);
        ++rejected_connections_;
        ::close(fd);
        continue;
      }
      serve(fd, ip);
      ::close(fd);
    }
    ::close(ls);
  }

  std::string status_json()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = steady_s();
    std::ostringstream o;
    o << "{\"enabled\":true,\"listen\":" << json_str(cfg_.listen_address + ":" + std::to_string(cfg_.listen_port))
      << ",\"state\":" << json_str(state_)
      << ",\"peer\":" << json_str(peer_)
      << ",\"connections\":" << connections_
      << ",\"rejected_connections\":" << rejected_connections_
      << ",\"protocol_errors\":" << protocol_errors_
      << ",\"last_error\":" << json_str(last_error_)
      << ",\"frames\":" << frames_
      << ",\"last_frame_age_s\":" << json_num(last_frame_ < 0 ? NAN : now - last_frame_)
      << ",\"tap_reported\":" << (tap_reported_.empty() ? "null" : tap_reported_)
      << ",\"tap_hello\":" << (hello_.empty() ? "null" : hello_) << "}";
    return o.str();
  }

private:
  void set_state(const std::string & s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = s;
  }

  void fail(const std::string & why)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++protocol_errors_;
    last_error_ = why;
  }

  // Reads exactly n bytes; false on EOF, error, stop or idle timeout.
  bool read_exact(int fd, void * buf, size_t n, double & last_activity)
  {
    auto * p = static_cast<uint8_t *>(buf);
    size_t got = 0;
    while (got < n) {
      if (g_stop) {return false;}
      pollfd pfd{fd, POLLIN, 0};
      int r = ::poll(&pfd, 1, 200);
      if (r == 0) {
        if (steady_s() - last_activity > cfg_.idle_timeout_s) {
          fail("idle timeout");
          return false;
        }
        continue;
      }
      if (r < 0) {return false;}
      ssize_t k = ::recv(fd, p + got, n - got, 0);
      if (k <= 0) {return false;}
      got += static_cast<size_t>(k);
      last_activity = steady_s();
    }
    return true;
  }

  static bool send_all(int fd, const void * buf, size_t n)
  {
    const auto * p = static_cast<const uint8_t *>(buf);
    while (n) {
      ssize_t k = ::send(fd, p, n, MSG_NOSIGNAL);
      if (k <= 0) {return false;}
      p += k;
      n -= static_cast<size_t>(k);
    }
    return true;
  }

  static bool send_frame(int fd, uint16_t kind, const std::string & header)
  {
    uint8_t prefix[16];
    std::memcpy(prefix, "S10L", 4);
    uint16_t version = 1;
    uint32_t hlen = static_cast<uint32_t>(header.size()), plen = 0;
    std::memcpy(prefix + 4, &version, 2);
    std::memcpy(prefix + 6, &kind, 2);
    std::memcpy(prefix + 8, &hlen, 4);
    std::memcpy(prefix + 12, &plen, 4);
    return send_all(fd, prefix, 16) && send_all(fd, header.data(), header.size());
  }

  void serve(int fd, const std::string & peer)
  {
    int rcvbuf = 8 << 20;
    ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      peer_ = peer;
      state_ = "connected";
      ++connections_;
    }
    ROS_INFO("tap: connected from %s", peer.c_str());
    double last_activity = steady_s();
    bool welcomed = false;
    while (!g_stop) {
      uint8_t prefix[16];
      if (!read_exact(fd, prefix, 16, last_activity)) {break;}
      uint16_t version = 0, kind = 0;
      uint32_t hlen = 0, plen = 0;
      std::memcpy(&version, prefix + 4, 2);
      std::memcpy(&kind, prefix + 6, 2);
      std::memcpy(&hlen, prefix + 8, 4);
      std::memcpy(&plen, prefix + 12, 4);
      if (std::memcmp(prefix, "S10L", 4) != 0 || version != 1) {fail("bad magic/version"); break;}
      if (hlen > (64u << 10) || plen > cfg_.max_payload_bytes) {fail("frame too large"); break;}
      std::string header(hlen, '\0');
      if (!read_exact(fd, header.data(), hlen, last_activity)) {break;}
      rclcpp::SerializedMessage payload(plen);
      auto & raw = payload.get_rcl_serialized_message();
      if (plen && !read_exact(fd, raw.buffer, plen, last_activity)) {break;}
      raw.buffer_length = plen;
      YAML::Node h;
      try {
        h = YAML::Load(header);
      } catch (const std::exception & e) {
        fail(std::string("bad header: ") + e.what());
        break;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++frames_;
        last_frame_ = steady_s();
      }
      if (kind == kHello) {
        std::ostringstream acc, rej;
        bool fa = true, fr = true;
        for (const auto & t : h["topics"]) {
          const auto topic = t["topic"].as<std::string>("");
          const auto type = t["type"].as<std::string>("");
          auto it = routes_.find(topic);
          const bool ok = it != routes_.end() && it->second->cfg().ros2_type == type;
          (ok ? acc : rej) << ((ok ? fa : fr) ? "" : ",") << json_str(topic);
          (ok ? fa : fr) = false;
        }
        std::string welcome = "{\"accepted\":[" + acc.str() + "],\"rejected\":[" + rej.str() +
          "],\"gateway\":" + json_str(kVersion) + "}";
        if (!send_frame(fd, kWelcome, welcome)) {break;}
        welcomed = true;
        std::lock_guard<std::mutex> lock(mutex_);
        hello_ = header;
      } else if (kind == kMessage) {
        if (!welcomed) {fail("message before hello"); break;}
        auto it = routes_.find(h["topic"].as<std::string>(""));
        if (it == routes_.end() || it->second->cfg().ros2_type != h["type"].as<std::string>("")) {
          fail("message for unknown topic/type");
          continue;
        }
        Meta m;
        m.recv_ns = h["recv_ns"].as<int64_t>(0);
        m.src_ns = h["src_ns"].as<int64_t>(0);
        m.pub_seq = h["pub_seq"].as<int64_t>(-1);
        m.tap_dropped = h["tap_dropped"].as<int64_t>(-1);
        it->second->forward(payload, m);
      } else if (kind == kHeartbeat) {
        std::lock_guard<std::mutex> lock(mutex_);
        tap_reported_ = header;
      } else {
        fail("unknown frame kind");
        break;
      }
    }
    ROS_WARN("tap: connection from %s closed", peer.c_str());
    set_state("listening");
  }

  TapConfig cfg_;
  std::map<std::string, std::shared_ptr<Route>> routes_;
  std::mutex mutex_;
  std::string state_ = "starting", peer_, last_error_, tap_reported_, hello_;
  uint64_t connections_ = 0, rejected_connections_ = 0, protocol_errors_ = 0, frames_ = 0;
  double last_frame_ = -1;
};

void write_status_file(const std::string & path, const std::string & text)
{
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {return;}
    out << text << "\n";
  }
  std::rename(tmp.c_str(), path.c_str());
}

void usage(const char * argv0)
{
  std::cerr << "usage: " << argv0 << " --config gateway.yaml --topics topics.yaml [--check]\n"
    "  --check  validate the configuration and ros1_bridge factories, then exit\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path, topics_path;
  bool check_only = false;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (a == "--topics" && i + 1 < argc) {
      topics_path = argv[++i];
    } else if (a == "--check") {
      check_only = true;
    } else {
      usage(argv[0]);
      return 2;
    }
  }
  if (config_path.empty() || topics_path.empty()) {usage(argv[0]); return 2;}

  GatewayConfig cfg;
  try {
    cfg = load_config(config_path, topics_path);
    for (const auto & t : cfg.topics) {
      if (!ros1_bridge::get_factory(t.ros1_type, t.ros2_type)) {
        throw std::runtime_error("no ros1_bridge factory for " + t.ros2_type);
      }
    }
  } catch (const std::exception & e) {
    std::cerr << "configuration error: " << e.what() << std::endl;
    return 2;
  }
  if (check_only) {
    for (const auto & t : cfg.topics) {
      auto f = ros1_bridge::get_factory(t.ros1_type, t.ros2_type);
      std::cout << t.source << "  " << t.ros2_topic << " [" << t.ros2_type << "] -> " << t.ros1_topic
                << " [" << f->get_ros1_data_type() << ", md5 " << f->get_ros1_md5sum() << "]\n";
    }
    std::cout << "CONFIG_OK\n";
    return 0;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // Only argv[0] is passed on: no remapping from the command line.
  int ros1_argc = 1;
  char * ros1_argv[] = {argv[0], nullptr};
  ros::init(ros1_argc, ros1_argv, cfg.ros1_node_name, ros::init_options::NoSigintHandler);
  while (!g_stop && !ros::master::check()) {
    ROS_WARN_THROTTLE(5, "waiting for ROS 1 master at %s", ros::master::getURI().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (g_stop) {return 0;}
  ros::NodeHandle nh;

  rclcpp::init(1, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<rclcpp::Node>(
    cfg.ros2_node_name,
    rclcpp::NodeOptions()
    .enable_rosout(false)
    .start_parameter_services(false)
    .start_parameter_event_publisher(false));

  std::vector<std::shared_ptr<Route>> routes;
  std::map<std::string, std::shared_ptr<Route>> tap_routes;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions;
  try {
    for (const auto & t : cfg.topics) {
      auto route = std::make_shared<Route>(t, nh);
      routes.push_back(route);
      if (t.source == "tap") {
        tap_routes[t.ros2_topic] = route;
        continue;
      }
      rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(t.ros2_depth))};
      if (t.ros2_reliability == "reliable") {qos.reliable();} else {qos.best_effort();}
      qos.durability_volatile();
      subscriptions.push_back(node->create_generic_subscription(
          t.ros2_topic, t.ros2_type, qos,
          [route](std::shared_ptr<const rclcpp::SerializedMessage> msg, const rclcpp::MessageInfo & info) {
            const auto & ri = info.get_rmw_message_info();
            Meta m;
            m.recv_ns = ri.received_timestamp;
            m.src_ns = ri.source_timestamp;
            m.pub_seq = static_cast<int64_t>(ri.publication_sequence_number);
            m.gid = hex_bytes(ri.publisher_gid.data, RMW_GID_STORAGE_SIZE);
            route->forward(*msg, m);
          }));
      ROS_INFO("route (dds) %s -> %s", t.ros2_topic.c_str(), t.ros1_topic.c_str());
    }
  } catch (const std::exception & e) {
    ROS_FATAL("startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros2_thread([&executor]() {executor.spin();});
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::unique_ptr<TapServer> tap;
  std::thread tap_thread;
  if (!tap_routes.empty() && cfg.tap.enabled) {
    tap = std::make_unique<TapServer>(cfg.tap, tap_routes);
    tap_thread = std::thread([&tap]() {
          try {
            tap->run();
          } catch (const std::exception & e) {
            ROS_ERROR("tap server stopped: %s", e.what());
          }
        });
  } else if (!tap_routes.empty()) {
    ROS_WARN("tap routes configured but tap.enabled is false: they will stay silent");
  }

  ros::Publisher status_pub;
  if (!cfg.status_topic.empty()) {status_pub = nh.advertise<std_msgs::String>(cfg.status_topic, 1);}
  const double started = steady_s();
  double next_status = started, next_log = started + 10;
  ROS_INFO("%s running: %zu routes, master %s", kVersion, routes.size(), ros::master::getURI().c_str());

  while (!g_stop && ros::ok() && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const double now = steady_s();
    if (now >= next_status) {
      next_status = now + cfg.status_period_s;
      std::ostringstream o;
      o << "{\"gateway\":" << json_str(kVersion) << ",\"pid\":" << ::getpid()
        << ",\"uptime_s\":" << json_num(now - started)
        << ",\"wall_ns\":" << wall_ns()
        << ",\"ros_master_uri\":" << json_str(ros::master::getURI())
        << ",\"routes\":[";
      for (size_t i = 0; i < routes.size(); ++i) {o << (i ? "," : "") << routes[i]->status_json();}
      o << "],\"tap\":" << (tap ? tap->status_json() : std::string("{\"enabled\":false}")) << "}";
      const std::string text = o.str();
      if (!cfg.status_file.empty()) {write_status_file(cfg.status_file, text);}
      if (status_pub) {
        std_msgs::String msg;
        msg.data = text;
        status_pub.publish(msg);
      }
    }
    if (now >= next_log) {
      next_log = now + 10;
      for (auto & r : routes) {ROS_INFO("%s", r->summary_line().c_str());}
    }
  }

  ROS_INFO("shutting down");
  g_stop = true;
  executor.cancel();
  if (ros2_thread.joinable()) {ros2_thread.join();}
  if (tap_thread.joinable()) {tap_thread.join();}
  spinner.stop();
  subscriptions.clear();
  rclcpp::shutdown();
  ros::shutdown();
  return 0;
}
