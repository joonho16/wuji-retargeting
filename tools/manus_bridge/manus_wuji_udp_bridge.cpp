// Standalone MANUS Core -> Wuji UDP bridge.
//
// This file is intentionally authored as project source instead of patching or
// vendoring MANUS SDK sample files. It builds against a locally mounted licensed
// MANUS SDK and emits MediaPipe-order 21x3 JSON frames consumed by
// example/input_devices/manus_quantum.py.

#include "ManusSDK.h"
#include "ManusSDKTypeInitializers.h"
#include "ManusSDKTypes.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

enum class ConnectionType {
  Integrated,
  Local,
  Remote,
};

struct RawSkeleton {
  RawSkeletonInfo info{};
  std::vector<SkeletonNode> nodes;
};

struct RawFrame {
  ManusTimestamp publish_time{};
  std::vector<RawSkeleton> skeletons;
};

std::atomic_bool g_running{true};
std::mutex g_frame_mutex;
std::optional<RawFrame> g_next_frame;

std::string getenv_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  return (value != nullptr && value[0] != '\0') ? std::string(value) : fallback;
}

std::string lowercase(std::string value) {
  for (char& c : value) {
    if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
  }
  return value;
}

bool sdk_ok(SDKReturnCode code, const char* step) {
  if (code == SDKReturnCode::SDKReturnCode_Success) return true;
  std::cerr << "MANUS SDK call failed at " << step << ": " << static_cast<int>(code) << std::endl;
  return false;
}

void handle_signal(int) {
  g_running.store(false);
}

void on_raw_skeleton_stream(const SkeletonStreamInfo* const stream_info) {
  if (stream_info == nullptr) return;

  RawFrame frame;
  frame.publish_time = stream_info->publishTime;
  frame.skeletons.resize(stream_info->skeletonsCount);

  for (uint32_t i = 0; i < stream_info->skeletonsCount; ++i) {
    RawSkeleton& skeleton = frame.skeletons[i];
    if (CoreSdk_GetRawSkeletonInfo(i, &skeleton.info) != SDKReturnCode::SDKReturnCode_Success) {
      continue;
    }
    skeleton.info.publishTime = stream_info->publishTime;
    skeleton.nodes.resize(skeleton.info.nodesCount);
    if (!skeleton.nodes.empty()) {
      CoreSdk_GetRawSkeletonData(i, skeleton.nodes.data(), skeleton.info.nodesCount);
    }
  }

  std::lock_guard<std::mutex> lock(g_frame_mutex);
  g_next_frame = std::move(frame);
}

ConnectionType parse_connection_type() {
  const std::string value = lowercase(getenv_or("MANUS_CONNECTION", "remote"));
  if (value == "1" || value == "integrated" || value == "core-integrated") return ConnectionType::Integrated;
  if (value == "2" || value == "local" || value == "core-local") return ConnectionType::Local;
  return ConnectionType::Remote;
}

const char* connection_type_name(ConnectionType type) {
  switch (type) {
    case ConnectionType::Integrated: return "integrated";
    case ConnectionType::Local: return "local";
    case ConnectionType::Remote: return "remote";
  }
  return "remote";
}

bool initialize_sdk(ConnectionType type) {
  const SDKReturnCode init_result = (type == ConnectionType::Integrated)
    ? CoreSdk_InitializeIntegrated()
    : CoreSdk_InitializeCore();
  if (!sdk_ok(init_result, "initialize")) return false;

  sdk_ok(CoreSdk_SetSessionType(SessionType::SessionType_CoreSDK), "set session type");

  if (!sdk_ok(CoreSdk_RegisterCallbackForRawSkeletonStream(on_raw_skeleton_stream),
              "register raw skeleton callback")) {
    return false;
  }

  CoordinateSystemVUH vuh;
  CoordinateSystemVUH_Init(&vuh);
  vuh.handedness = Side::Side_Right;
  vuh.up = AxisPolarity::AxisPolarity_PositiveZ;
  vuh.view = AxisView::AxisView_XFromViewer;
  vuh.unitScale = 1.0f;
  if (!sdk_ok(CoreSdk_InitializeCoordinateSystemWithVUH(vuh, true), "initialize coordinate system")) {
    return false;
  }

  return true;
}

void rewrite_discovered_hosts_for_unicast(std::vector<ManusHost>& hosts, bool local_connection) {
  const char* core_ip = std::getenv("MANUS_CORE_IP");
  if (local_connection || core_ip == nullptr || core_ip[0] == '\0') return;

  for (ManusHost& host : hosts) {
    const std::string original(host.ipAddress);
    std::string rewritten(core_ip);
    const std::size_t port_pos = original.rfind(':');
    if (port_pos != std::string::npos) rewritten += original.substr(port_pos);

    std::cout << "Using MANUS_CORE_IP=" << core_ip
              << " for discovered host " << host.hostName
              << " at " << original << " -> " << rewritten << std::endl;
    std::strncpy(host.ipAddress, rewritten.c_str(), sizeof(host.ipAddress) - 1);
    host.ipAddress[sizeof(host.ipAddress) - 1] = '\0';
  }
}

bool connect_to_core(ConnectionType type) {
  const bool local_connection = type == ConnectionType::Local;

  while (g_running.load()) {
    if (!sdk_ok(CoreSdk_LookForHosts(3, local_connection), "look for hosts")) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    uint32_t host_count = 0;
    if (!sdk_ok(CoreSdk_GetNumberOfAvailableHostsFound(&host_count), "get host count") || host_count == 0) {
      std::cerr << "No MANUS Core hosts found; retrying in 1s." << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }

    std::vector<ManusHost> hosts(host_count);
    if (!sdk_ok(CoreSdk_GetAvailableHostsFound(hosts.data(), host_count), "get hosts")) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    rewrite_discovered_hosts_for_unicast(hosts, local_connection);

    for (uint32_t i = 0; i < host_count; ++i) {
      std::cout << "MANUS host [" << i << "]: " << hosts[i].hostName
                << " " << hosts[i].ipAddress << std::endl;
    }

    const SDKReturnCode connect_result = CoreSdk_ConnectToHost(hosts[0]);
    if (connect_result == SDKReturnCode::SDKReturnCode_Success) {
      std::cout << "Connected to MANUS Core via " << hosts[0].ipAddress << std::endl;
      return true;
    }
    std::cerr << "Could not connect to MANUS Core (" << static_cast<int>(connect_result)
              << "); retrying in 1s." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return false;
}

class UdpPublisher {
 public:
  UdpPublisher(std::string host, uint16_t port, std::string hand)
      : host_(std::move(host)), port_(port), hand_(std::move(hand)) {}

  bool open_socket() {
    fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd_ < 0) {
      perror("socket");
      return false;
    }

    std::memset(&addr_, 0, sizeof(addr_));
    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(port_);
    if (inet_pton(AF_INET, host_.c_str(), &addr_.sin_addr) != 1) {
      std::cerr << "Invalid MANUS_UDP_HOST (IPv4 literal required): " << host_ << std::endl;
      close(fd_);
      fd_ = -1;
      return false;
    }

    std::cout << "MANUS UDP bridge target: " << host_ << ":" << port_
              << " hand=" << hand_ << std::endl;
    return true;
  }

  ~UdpPublisher() {
    if (fd_ >= 0) close(fd_);
  }

  bool publish(const std::array<ManusVec3, 21>& keypoints) const {
    if (fd_ < 0) return false;
    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{\"hand\":\"" << hand_ << "\",\"keypoints\":[";
    for (std::size_t i = 0; i < keypoints.size(); ++i) {
      if (i != 0) json << ',';
      json << '[' << keypoints[i].x << ',' << keypoints[i].y << ',' << keypoints[i].z << ']';
    }
    json << "]}";
    const std::string payload = json.str();
    const ssize_t sent = sendto(fd_, payload.data(), payload.size(), 0,
                                reinterpret_cast<const sockaddr*>(&addr_), sizeof(addr_));
    return sent == static_cast<ssize_t>(payload.size());
  }

  const std::string& hand() const { return hand_; }

 private:
  std::string host_;
  uint16_t port_;
  std::string hand_;
  int fd_ = -1;
  sockaddr_in addr_{};
};

bool find_node_position(const RawSkeleton& skeleton, uint32_t node_id, ManusVec3& out) {
  for (const SkeletonNode& node : skeleton.nodes) {
    if (node.id == node_id) {
      out = node.transform.position;
      return true;
    }
  }
  return false;
}

bool build_mediapipe_keypoints(const RawSkeleton& skeleton,
                               const std::string& hand,
                               std::array<ManusVec3, 21>& keypoints) {
  for (ManusVec3& keypoint : keypoints) keypoint = ManusVec3{0.0f, 0.0f, 0.0f};

  uint32_t node_count = 0;
  if (CoreSdk_GetRawSkeletonNodeCount(skeleton.info.gloveId, node_count) != SDKReturnCode::SDKReturnCode_Success ||
      node_count == 0) {
    return false;
  }

  std::vector<NodeInfo> node_info(node_count);
  if (CoreSdk_GetRawSkeletonNodeInfoArray(skeleton.info.gloveId, node_info.data(), node_count) != SDKReturnCode::SDKReturnCode_Success) {
    return false;
  }

  const Side desired_side = (hand == "right") ? Side::Side_Right : Side::Side_Left;
  bool matched_side = false;
  bool has_hand_root = false;

  auto set_keypoint = [&](uint32_t node_id, std::size_t index) {
    if (index >= keypoints.size()) return;
    ManusVec3 position;
    if (find_node_position(skeleton, node_id, position)) keypoints[index] = position;
  };

  for (const NodeInfo& info : node_info) {
    if (info.side != desired_side) continue;
    matched_side = true;

    if (info.chainType == ChainType::ChainType_Hand) {
      set_keypoint(info.nodeId, 0);
      has_hand_root = true;
      continue;
    }

    std::size_t base = 0;
    switch (info.chainType) {
      case ChainType::ChainType_FingerThumb:  base = 1; break;
      case ChainType::ChainType_FingerIndex:  base = 5; break;
      case ChainType::ChainType_FingerMiddle: base = 9; break;
      case ChainType::ChainType_FingerRing:   base = 13; break;
      case ChainType::ChainType_FingerPinky:  base = 17; break;
      default: continue;
    }

    if (info.chainType == ChainType::ChainType_FingerThumb) {
      switch (info.fingerJointType) {
        case FingerJointType::FingerJointType_Metacarpal: set_keypoint(info.nodeId, base + 0); break;
        case FingerJointType::FingerJointType_Proximal:   set_keypoint(info.nodeId, base + 1); break;
        case FingerJointType::FingerJointType_Distal:     set_keypoint(info.nodeId, base + 2); break;
        case FingerJointType::FingerJointType_Tip:        set_keypoint(info.nodeId, base + 3); break;
        default: break;
      }
    } else {
      // MANUS non-thumb fingers expose a metacarpal plus four distal nodes.
      // Wuji consumes MediaPipe order, so skip metacarpal and publish
      // Proximal/Intermediate/Distal/Tip as MCP/PIP/DIP/TIP.
      switch (info.fingerJointType) {
        case FingerJointType::FingerJointType_Proximal:     set_keypoint(info.nodeId, base + 0); break;
        case FingerJointType::FingerJointType_Intermediate: set_keypoint(info.nodeId, base + 1); break;
        case FingerJointType::FingerJointType_Distal:       set_keypoint(info.nodeId, base + 2); break;
        case FingerJointType::FingerJointType_Tip:          set_keypoint(info.nodeId, base + 3); break;
        default: break;
      }
    }
  }

  return matched_side && has_hand_root;
}

bool publish_frame(const RawFrame& frame, const UdpPublisher& publisher) {
  std::array<ManusVec3, 21> keypoints;
  for (const RawSkeleton& skeleton : frame.skeletons) {
    if (build_mediapipe_keypoints(skeleton, publisher.hand(), keypoints)) {
      return publisher.publish(keypoints);
    }
  }
  return false;
}

void print_help(const char* argv0) {
  std::cout << "Usage: " << argv0 << "\n\n"
            << "Environment:\n"
            << "  MANUS_CONNECTION=remote|local|integrated  default: remote\n"
            << "  MANUS_CORE_IP=<ip>                         optional host rewrite for remote/unicast discovery\n"
            << "  MANUS_UDP_HOST=127.0.0.1                   UDP target host\n"
            << "  MANUS_UDP_PORT=15020                       UDP target port\n"
            << "  MANUS_UDP_HAND=left|right                  hand key in JSON payload\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    print_help(argv[0]);
    return 0;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  const ConnectionType connection_type = parse_connection_type();
  std::string hand = lowercase(getenv_or("MANUS_UDP_HAND", getenv_or("HAND", "left")));
  if (hand != "right") hand = "left";
  const std::string udp_host = getenv_or("MANUS_UDP_HOST", "127.0.0.1");
  const uint16_t udp_port = static_cast<uint16_t>(std::stoi(getenv_or("MANUS_UDP_PORT", "15020")));

  std::cout << "Starting tracked MANUS -> Wuji UDP bridge"
            << " connection=" << connection_type_name(connection_type)
            << " hand=" << hand << std::endl;

  if (!initialize_sdk(connection_type)) return 10;

  UdpPublisher publisher(udp_host, udp_port, hand);
  if (!publisher.open_socket()) {
    CoreSdk_ShutDown();
    return 11;
  }

  if (!connect_to_core(connection_type)) {
    CoreSdk_ShutDown();
    return 12;
  }

  sdk_ok(CoreSdk_SetRawSkeletonHandMotion(HandMotion::HandMotion_Auto), "set raw skeleton hand motion");

  uint64_t frame_count = 0;
  auto last_wait_log = std::chrono::steady_clock::now();
  while (g_running.load()) {
    std::optional<RawFrame> frame;
    {
      std::lock_guard<std::mutex> lock(g_frame_mutex);
      frame = std::move(g_next_frame);
      g_next_frame.reset();
    }

    if (frame.has_value()) {
      if (publish_frame(*frame, publisher)) {
        ++frame_count;
        if (frame_count == 1 || frame_count % 120 == 0) {
          std::cout << "Published MANUS UDP frames: " << frame_count << std::endl;
        }
      }
    } else {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_wait_log > std::chrono::seconds(5)) {
        std::cout << "Waiting for raw MANUS skeleton frames..." << std::endl;
        last_wait_log = now;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  CoreSdk_ShutDown();
  std::cout << "MANUS UDP bridge stopped." << std::endl;
  return 0;
}
