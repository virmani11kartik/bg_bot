// imu_udp_tx.cpp
#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <vector>

static constexpr const char* PORT = "/dev/ttymxc4";
static constexpr int BAUD = 115200;

static constexpr const char* DEST_IP = "192.168.10.20";
static constexpr int DEST_PORT = 5005;

// ---------------- Serial helpers ----------------
static speed_t baud_to_termios(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

static int open_serial(const char* device, int baud) {
  int fd = ::open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) { std::perror("open serial"); return -1; }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) { std::perror("tcgetattr"); ::close(fd); return -1; }

  cfmakeraw(&tty);
  speed_t spd = baud_to_termios(baud);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) { std::perror("tcsetattr"); ::close(fd); return -1; }
  return fd;
}

static bool wait_readable(int fd, int timeout_ms) {
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd, &set);
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  int r = select(fd + 1, &set, nullptr, nullptr, &tv);
  return r > 0 && FD_ISSET(fd, &set);
}

static bool read_exact(int fd, uint8_t* buf, size_t n, int timeout_ms) {
  size_t got = 0;
  while (got < n) {
    if (!wait_readable(fd, timeout_ms)) return false;
    ssize_t r = ::read(fd, buf + got, n - got);
    if (r < 0) { if (errno == EINTR) continue; std::perror("read"); return false; }
    if (r == 0) continue;
    got += static_cast<size_t>(r);
  }
  return true;
}

static bool write_all(int fd, const uint8_t* buf, size_t n) {
  size_t sent = 0;
  while (sent < n) {
    ssize_t w = ::write(fd, buf + sent, n - sent);
    if (w < 0) { if (errno == EINTR) continue; std::perror("write"); return false; }
    sent += static_cast<size_t>(w);
  }
  return true;
}

static uint8_t checksum8(const std::vector<uint8_t>& pkt) {
  uint32_t s = 0;
  for (size_t i = 1; i < pkt.size(); i++) s += pkt[i];
  return static_cast<uint8_t>((256 - (s & 0xFF)) & 0xFF);
}

static bool send_packet(int fd, uint8_t mid, const std::vector<uint8_t>& data = {}) {
  std::vector<uint8_t> pkt;
  pkt.reserve(4 + data.size() + 1);
  pkt.push_back(0xFA);
  pkt.push_back(0xFF);
  pkt.push_back(mid);
  pkt.push_back(static_cast<uint8_t>(data.size()));
  pkt.insert(pkt.end(), data.begin(), data.end());
  pkt.push_back(checksum8(pkt));
  if (!write_all(fd, pkt.data(), pkt.size())) return false;
  usleep(100000);
  return true;
}

static float read_be_float(const uint8_t* p) {
  uint32_t u = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
  float f;
  std::memcpy(&f, &u, 4);
  return f;
}

// ---------------- UDP + CRC ----------------
static uint64_t monotonic_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return uint64_t(ts.tv_sec) * 1000000000ULL + uint64_t(ts.tv_nsec);
}

// Simple CRC32 (Ethernet polynomial). Good enough for packets.
static uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

#pragma pack(push, 1)
struct UdpImuPacket {
  uint32_t magic;        // "UIMU" = 0x554D4955
  uint16_t version;      // 1
  uint16_t payload_len;  // sizeof(UdpImuPacket)
  uint32_t seq;
  uint64_t t_monotonic_ns;
  float roll;
  float pitch;
  float yaw;
  uint32_t crc;
};
#pragma pack(pop)

int main() {
  int sfd = open_serial(PORT, BAUD);
  if (sfd < 0) return 1;

  // Configure IMU like your code: config mode -> set Euler @ 100Hz -> measurement mode
  if (!send_packet(sfd, 0x30)) return 1;
  std::vector<uint8_t> cfg = {0x20, 0x30, 0x00, 0x64};
  if (!send_packet(sfd, 0xC0, cfg)) return 1;
  if (!send_packet(sfd, 0x10)) return 1;

  // UDP socket
  int ufd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (ufd < 0) { std::perror("socket"); return 1; }

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(DEST_PORT);
  if (inet_pton(AF_INET, DEST_IP, &dst.sin_addr) != 1) {
    std::fprintf(stderr, "inet_pton failed for %s\n", DEST_IP);
    return 1;
  }

  std::printf("Streaming IMU Euler over UDP -> %s:%d\n", DEST_IP, DEST_PORT);
  uint32_t seq = 0;

  while (true) {
    // Find preamble 0xFA 0xFF
    uint8_t b = 0;
    if (!read_exact(sfd, &b, 1, 1000)) continue;
    if (b != 0xFA) continue;
    if (!read_exact(sfd, &b, 1, 1000)) continue;
    if (b != 0xFF) continue;

    uint8_t mid = 0, length = 0;
    if (!read_exact(sfd, &mid, 1, 1000)) continue;
    if (!read_exact(sfd, &length, 1, 1000)) continue;

    std::vector<uint8_t> payload(length);
    if (!read_exact(sfd, payload.data(), payload.size(), 1000)) continue;

    uint8_t rx_csum = 0;
    if (!read_exact(sfd, &rx_csum, 1, 1000)) continue;

    if (mid != 0x36) continue; // MTData2 only

    // Parse blocks
    size_t i = 0;
    while (i + 3 <= payload.size()) {
      uint16_t did = (uint16_t(payload[i]) << 8) | uint16_t(payload[i + 1]);
      uint8_t size = payload[i + 2];
      if (i + 3 + size > payload.size()) break;

      const uint8_t* data = payload.data() + i + 3;

      if (did == 0x2030 && size == 12) {
        float roll  = read_be_float(data + 0);
        float pitch = read_be_float(data + 4);
        float yaw   = read_be_float(data + 8);

        UdpImuPacket pkt{};
        pkt.magic = 0x554D4955u; // "UIMU"
        pkt.version = 1;
        pkt.payload_len = sizeof(UdpImuPacket);
        pkt.seq = seq++;
        pkt.t_monotonic_ns = monotonic_ns();
        pkt.roll = roll;
        pkt.pitch = pitch;
        pkt.yaw = yaw;

        pkt.crc = 0;
        pkt.crc = crc32(reinterpret_cast<const uint8_t*>(&pkt), sizeof(UdpImuPacket) - sizeof(uint32_t));

        ssize_t sent = sendto(ufd, &pkt, sizeof(pkt), 0,
                              reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
        if (sent != (ssize_t)sizeof(pkt)) {
          std::perror("sendto");
        }
      }

      i += 3 + size;
    }
  }

  ::close(ufd);
  ::close(sfd);
  return 0;
}
