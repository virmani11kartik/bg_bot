// speed_test.cpp
//
// C++ version of speed_test.py
// - Sends NMT (pre-op, start)
// - Talks to one node via SDO reads/writes
// - Sets CiA402 Profile Velocity mode (6060=3)
// - Enables drive via controlword sequence
// - Ramps 60FF to a target, holds while reading 606C, then ramps down and disables
//
// Build:
//   g++ -O2 -std=c++17 speed_test.cpp -o speed_test

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

static const char* CHANNEL = "can0";
static const int   NODE    = 2;     // set to -1 for "auto-detect" (optional code included)

static constexpr double CMD_PER_WHEEL_RPM = 3814.0;
static constexpr double TARGET_WHEEL_RPM  = -80.0;
static constexpr double HOLD_SEC          = 5.0;

static constexpr int32_t RAMP_STEP_CMD = 15000;
static constexpr double  RAMP_DT       = 0.12;

static constexpr bool   PRINT_606C = true;
static constexpr double TELEM_DT   = 0.5;

// ------------------------- small time helpers -------------------------
static inline double now_sec() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

static inline void sleep_sec(double s) {
  if (s <= 0) return;
  std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

// ------------------------- SocketCAN wrapper -------------------------
class CanSocket {
public:
  explicit CanSocket(const std::string& ifname) {
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) throw std::runtime_error("socket(PF_CAN) failed");

    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
      ::close(fd_);
      throw std::runtime_error("ioctl(SIOCGIFINDEX) failed for " + ifname);
    }

    struct sockaddr_can addr {};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      ::close(fd_);
      throw std::runtime_error("bind(AF_CAN) failed for " + ifname);
    }
  }

  ~CanSocket() {
    if (fd_ >= 0) ::close(fd_);
  }

  void send_frame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (len > 8) throw std::runtime_error("CAN len > 8 not supported");
    struct can_frame f {};
    f.can_id  = can_id & CAN_SFF_MASK; // standard 11-bit
    f.can_dlc = len;
    if (len) std::memcpy(f.data, data, len);

    ssize_t n = ::write(fd_, &f, sizeof(f));
    if (n != (ssize_t)sizeof(f)) throw std::runtime_error("write(can_frame) failed");
  }

  // Receive with timeout. Returns true if a frame was read.
  bool recv_frame(struct can_frame& out, double timeout_sec) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd_, &rfds);

    struct timeval tv;
    tv.tv_sec  = (int)timeout_sec;
    tv.tv_usec = (int)((timeout_sec - tv.tv_sec) * 1e6);

    int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (r < 0) throw std::runtime_error("select() failed");
    if (r == 0) return false; // timeout

    ssize_t n = ::read(fd_, &out, sizeof(out));
    if (n != (ssize_t)sizeof(out)) return false;
    return true;
  }

private:
  int fd_ = -1;
};

// ------------------------- CANopen helpers -------------------------
static inline void nmt(CanSocket& bus, uint8_t cmd, uint8_t node_id /*0=broadcast*/) {
  // NMT uses CAN-ID 0x000, data [cmd, node]
  uint8_t d[2] = { (uint8_t)(cmd & 0xFF), (uint8_t)(node_id & 0xFF) };
  bus.send_frame(0x000, d, 2);
}

static inline int32_t le_i32(const uint8_t* p) {
  int32_t v;
  std::memcpy(&v, p, 4);
  // CANopen SDO uses little-endian; on x86 this is already LE.
  return v;
}

static inline uint16_t le_u16(const uint8_t* p) {
  uint16_t v;
  std::memcpy(&v, p, 2);
  return v;
}

static inline void store_le_u16(uint8_t* p, uint16_t v) {
  std::memcpy(p, &v, 2);
}

static inline void store_le_i32(uint8_t* p, int32_t v) {
  std::memcpy(p, &v, 4);
}

class Ctrl {
public:
  Ctrl(CanSocket& bus, int node) : bus_(bus), node_(node) {}

  // Wait for matching SDO response to idx/sub on 0x580+node
  bool wait_sdo(uint16_t idx, uint8_t sub, struct can_frame& out, double timeout_sec = 0.5) {
    const uint32_t cob = 0x580 + (uint32_t)node_;
    const double t0 = now_sec();
    while ((now_sec() - t0) < timeout_sec) {
      struct can_frame f {};
      double remain = timeout_sec - (now_sec() - t0);
      if (remain < 0) remain = 0;
      if (!bus_.recv_frame(f, remain)) continue;

      if ((f.can_id & CAN_SFF_MASK) != cob) continue;
      if (f.can_dlc < 8) continue;

      // index/sub match in bytes 1..3 of SDO response
      if (f.data[1] == (idx & 0xFF) &&
          f.data[2] == ((idx >> 8) & 0xFF) &&
          f.data[3] == sub) {
        out = f;
        return true;
      }
    }
    return false;
  }

  // SDO read i32
  bool r_i32(uint16_t idx, uint8_t sub, int32_t& out_val) {
    uint8_t d[8] = {0};
    d[0] = 0x40; // initiate upload (read)
    d[1] = (uint8_t)(idx & 0xFF);
    d[2] = (uint8_t)((idx >> 8) & 0xFF);
    d[3] = sub;
    bus_.send_frame(0x600 + (uint32_t)node_, d, 8);

    struct can_frame r {};
    if (!wait_sdo(idx, sub, r, 0.5)) return false;

    if (r.data[0] == 0x80) return false; // abort
    out_val = le_i32(&r.data[4]);
    return true;
  }

  // SDO write u16
  void w_u16(uint16_t idx, uint8_t sub, uint16_t val) {
    uint8_t d[8] = {0};
    d[0] = 0x2B; // expedited download, 2 bytes
    d[1] = (uint8_t)(idx & 0xFF);
    d[2] = (uint8_t)((idx >> 8) & 0xFF);
    d[3] = sub;
    store_le_u16(&d[4], val);
    bus_.send_frame(0x600 + (uint32_t)node_, d, 8);

    struct can_frame r {};
    if (wait_sdo(idx, sub, r, 0.5) && r.data[0] == 0x80) {
      uint32_t abort = (uint32_t)le_i32(&r.data[4]);
      throw std::runtime_error("SDO abort write " + hex_idx(idx, sub, abort));
    }
  }

  // SDO write i32
  void w_i32(uint16_t idx, uint8_t sub, int32_t val) {
    uint8_t d[8] = {0};
    d[0] = 0x23; // expedited download, 4 bytes
    d[1] = (uint8_t)(idx & 0xFF);
    d[2] = (uint8_t)((idx >> 8) & 0xFF);
    d[3] = sub;
    store_le_i32(&d[4], val);
    bus_.send_frame(0x600 + (uint32_t)node_, d, 8);

    struct can_frame r {};
    if (wait_sdo(idx, sub, r, 0.5) && r.data[0] == 0x80) {
      uint32_t abort = (uint32_t)le_i32(&r.data[4]);
      throw std::runtime_error("SDO abort write " + hex_idx(idx, sub, abort));
    }
  }

  void controlword(uint16_t v) { w_u16(0x6040, 0x00, v); }

  void set_mode_velocity() {
    // write int8 value 0x03 to 6060:00 using 1-byte expedited download (0x2F)
    uint8_t d[8] = {0};
    d[0] = 0x2F; // expedited download, 1 byte
    d[1] = 0x60;
    d[2] = 0x60;
    d[3] = 0x00;
    d[4] = 0x03;
    bus_.send_frame(0x600 + (uint32_t)node_, d, 8);

    struct can_frame r {};
    if (wait_sdo(0x6060, 0x00, r, 0.5) && r.data[0] == 0x80) {
      uint32_t abort = (uint32_t)le_i32(&r.data[4]);
      throw std::runtime_error("SDO abort write 6060:00 0x" + hex_u32(abort));
    }
  }

  void cmd(int32_t v) { w_i32(0x60FF, 0x00, v); }

  bool sw(int32_t& v) { return r_i32(0x6041, 0x00, v); }
  bool md(int32_t& v) { return r_i32(0x6061, 0x00, v); }
  bool av(int32_t& v) { return r_i32(0x606C, 0x00, v); }

  void enable() {
    controlword(0x0006); sleep_sec(0.10);
    set_mode_velocity(); sleep_sec(0.10);
    controlword(0x0007); sleep_sec(0.10);
    controlword(0x000F); sleep_sec(0.20);
  }

  void disable() {
    controlword(0x0007); sleep_sec(0.05);
    controlword(0x0006); sleep_sec(0.05);
  }

private:
  static std::string hex_u32(uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%08X", v);
    return std::string(buf);
  }

  static std::string hex_idx(uint16_t idx, uint8_t sub, uint32_t abort) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04X:%02X 0x%08X", idx, sub, abort);
    return std::string(buf);
  }

  CanSocket& bus_;
  int node_;
};

// simple SDO ping to 6061:00, for node autodetect
static bool sdo_ping(CanSocket& bus, int node, double timeout_sec = 0.2) {
  uint8_t d[8] = {0};
  d[0] = 0x40; // read
  d[1] = 0x61;
  d[2] = 0x60;
  d[3] = 0x00;
  bus.send_frame(0x600 + (uint32_t)node, d, 8);

  const uint32_t resp = 0x580 + (uint32_t)node;
  const double t0 = now_sec();
  while ((now_sec() - t0) < timeout_sec) {
    struct can_frame f {};
    double remain = timeout_sec - (now_sec() - t0);
    if (remain < 0) remain = 0;
    if (!bus.recv_frame(f, remain)) continue;
    if ((f.can_id & CAN_SFF_MASK) == resp && f.can_dlc >= 8) return true;
  }
  return false;
}

static int autodetect_node(CanSocket& bus, int lo = 1, int hi = 10) {
  for (int n = lo; n <= hi; n++) {
    if (sdo_ping(bus, n, 0.2)) return n;
  }
  return -1;
}

static int32_t wheel_rpm_to_cmd(double rpm) {
  return (int32_t)std::llround(rpm * CMD_PER_WHEEL_RPM);
}

static void ramp(Ctrl& ctrl, int32_t start, int32_t target, int32_t step, double dt) {
  int32_t v = start;
  if (target >= start) {
    while (v < target) {
      v = std::min<int32_t>(v + step, target);
      ctrl.cmd(v);
      sleep_sec(dt);
    }
  } else {
    while (v > target) {
      v = std::max<int32_t>(v - step, target);
      ctrl.cmd(v);
      sleep_sec(dt);
    }
  }
}

int main() {
  try {
    CanSocket bus(CHANNEL);

    // Bring network up after power-cycle
    nmt(bus, 0x80, 0); // pre-op all
    sleep_sec(0.10);
    nmt(bus, 0x01, 0); // start all
    sleep_sec(0.10);

    int node = NODE;
    if (node < 0) {
      node = autodetect_node(bus, 1, 10);
      if (node < 0) throw std::runtime_error("No SDO responses from nodes 1..10.");
    }
    std::cout << "Using NODE = " << node << "\n";

    Ctrl ctrl(bus, node);

    const int32_t target_cmd = wheel_rpm_to_cmd(TARGET_WHEEL_RPM);
    std::cout << "TARGET_WHEEL_RPM=" << TARGET_WHEEL_RPM
              << " -> target_cmd=" << target_cmd
              << " (CMD_PER_WHEEL_RPM=" << CMD_PER_WHEEL_RPM << ")\n";

    ctrl.enable();

    int32_t md = 0, sw = 0;
    bool ok_md = ctrl.md(md);
    bool ok_sw = ctrl.sw(sw);
    std::cout << "Mode display (6061): " << (ok_md ? std::to_string(md) : "None") << "\n";
    std::cout << "Statusword (6041):   " << (ok_sw ? std::to_string(sw) : "None") << "\n";

    if (!ok_md || !ok_sw) {
      throw std::runtime_error("Drive not answering SDO reads after enable. Check node ID/NMT/bitrate/state.");
    }

    // Run
    ctrl.cmd(0);
    sleep_sec(0.10);

    std::cout << "Ramping up...\n";
    ramp(ctrl, 0, target_cmd, RAMP_STEP_CMD, RAMP_DT);

    std::cout << "Holding for " << HOLD_SEC << "s...\n";
    std::vector<int32_t> samples;
    const double t0 = now_sec();
    while ((now_sec() - t0) < HOLD_SEC) {
      if (PRINT_606C) {
        int32_t av = 0;
        bool ok = ctrl.av(av);
        std::cout << "  606C actual vel: " << (ok ? std::to_string(av) : "None") << "\n";
        if (ok) samples.push_back(av);
      }
      sleep_sec(TELEM_DT);
    }

    if (PRINT_606C && !samples.empty()) {
      double mean = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
      double var = 0.0;
      for (double x : samples) var += (x - mean) * (x - mean);
      var /= samples.size(); // population variance
      double stddev = std::sqrt(var);
      std::cout << "606C avg/std: " << mean << " / " << stddev << "\n";
    }

    std::cout << "Ramping down...\n";
    ramp(ctrl, target_cmd, 0, RAMP_STEP_CMD, std::max(0.08, RAMP_DT * 0.9));

    // Safety stop
    ctrl.cmd(0);
    sleep_sec(0.10);
    ctrl.disable();

    std::cout << "Done.\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
