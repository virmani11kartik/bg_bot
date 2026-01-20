#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <string>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>

static constexpr const char* PORT = "/dev/ttymxc4";
static constexpr int BAUD = 115200;

// -------------------- Serial helpers --------------------

static speed_t baud_to_termios(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200; // fallback
  }
}


static int open_serial(const char* device, int baud) {
  int fd = ::open(device, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    std::perror("open");
    return -1;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    std::perror("tcgetattr");
    ::close(fd);
    return -1;
  }

  // Raw mode-ish
  cfmakeraw(&tty);

  speed_t spd = baud_to_termios(baud);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_cflag |= (CLOCAL | CREAD);            // ignore modem controls, enable read
  tty.c_cflag &= ~(PARENB | PARODD);          // no parity
  tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;                    // no HW flow control

  // Non-blocking behavior controlled with select() below,
  // but keep VMIN/VTIME at 0 for pure non-blocking read.
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::perror("tcsetattr");
    ::close(fd);
    return -1;
  }

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
    if (r < 0) {
      if (errno == EINTR) continue;
      std::perror("read");
      return false;
    }
    if (r == 0) continue; // no data yet
    got += static_cast<size_t>(r);
  }
  return true;
}

static bool write_all(int fd, const uint8_t* buf, size_t n) {
  size_t sent = 0;
  while (sent < n) {
    ssize_t w = ::write(fd, buf + sent, n - sent);
    if (w < 0) {
      if (errno == EINTR) continue;
      std::perror("write");
      return false;
    }
    sent += static_cast<size_t>(w);
  }
  return true;
}

// -------------------- Protocol helpers --------------------

static uint8_t checksum8(const std::vector<uint8_t>& pkt) {
  // matches Python: (256 - (sum(pkt[1:]) & 0xFF)) & 0xFF
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

  uint8_t csum = checksum8(pkt);
  pkt.push_back(csum);

  if (!write_all(fd, pkt.data(), pkt.size())) return false;
  usleep(100000); // 0.1s 
  return true;
}

static float read_be_float(const uint8_t* p) {
  // Convert 4 bytes big-endian to float
  uint32_t u = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
  float f;
  static_assert(sizeof(float) == 4, "float must be 4 bytes");
  std::memcpy(&f, &u, 4);
  return f;
}


int main() {
  int fd = open_serial(PORT, BAUD);
  if (fd < 0) return 1;

  // 1) Go to config mode (MID 0x30)
  if (!send_packet(fd, 0x30)) return 1;

  // 2) Set output configuration: Euler angles @ 100 Hz
  // Python payload: "20 30 00 64"
  std::vector<uint8_t> cfg = {0x20, 0x30, 0x00, 0x64};
  if (!send_packet(fd, 0xC0, cfg)) return 1;

  // 3) Go to measurement mode (MID 0x10)
  if (!send_packet(fd, 0x10)) return 1;

  std::printf("Streaming Euler angles (roll, pitch, yaw)...\n");

  while (true) {
    // Scan for preamble 0xFA 0xFF
    uint8_t b = 0;
    if (!read_exact(fd, &b, 1, 1000)) continue;
    if (b != 0xFA) continue;

    if (!read_exact(fd, &b, 1, 1000)) continue;
    if (b != 0xFF) continue;

    uint8_t mid = 0, length = 0;
    if (!read_exact(fd, &mid, 1, 1000)) continue;
    if (!read_exact(fd, &length, 1, 1000)) continue;

    std::vector<uint8_t> payload(length);
    if (!read_exact(fd, payload.data(), payload.size(), 1000)) continue;

    uint8_t rx_csum = 0;
    if (!read_exact(fd, &rx_csum, 1, 1000)) continue;

    // Only MTData2
    if (mid != 0x36) continue;

    // Parse blocks: [DID(2)][SIZE(1)][DATA(SIZE)]
    size_t i = 0;
    while (i + 3 <= payload.size()) {
      uint16_t did = (uint16_t(payload[i]) << 8) | uint16_t(payload[i + 1]);
      uint8_t size = payload[i + 2];

      if (i + 3 + size > payload.size()) break; // malformed/truncated
      const uint8_t* data = payload.data() + i + 3;

      if (did == 0x2030 && size == 12) { // 3 floats = 12 bytes
        float roll  = read_be_float(data + 0);
        float pitch = read_be_float(data + 4);
        float yaw   = read_be_float(data + 8);
        std::printf("ROLL % .3f  PITCH % .3f  YAW % .3f\n", roll, pitch, yaw);
        std::fflush(stdout);
      }

      i += 3 + size;
    }
  }

  ::close(fd);
  return 0;
}