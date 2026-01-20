#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

static constexpr const char* PORT = "/dev/ttymxc4";
static constexpr int BAUD = 115200;

// ------------------------- Serial helpers -------------------------

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

static int open_serial(const std::string& dev, int baud) {
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    std::cerr << "Failed to open " << dev << ": " << std::strerror(errno) << "\n";
    return -1;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    std::cerr << "tcgetattr failed: " << std::strerror(errno) << "\n";
    ::close(fd);
    return -1;
  }

  // Raw-ish 8N1
  cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
  tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem controls, enable reading
  tty.c_cflag &= ~(PARENB | PARODD);           // no parity
  tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;                     // no HW flow control

  // Timeouts: similar idea to Python timeout=1
  // VTIME is in deciseconds. VMIN=0 means "return as soon as any data is available or timeout"
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 10; // 1.0 seconds

  speed_t spd = baud_to_termios(baud);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cerr << "tcsetattr failed: " << std::strerror(errno) << "\n";
    ::close(fd);
    return -1;
  }

  return fd;
}

static bool write_all(int fd, const uint8_t* data, size_t n) {
  while (n > 0) {
    ssize_t w = ::write(fd, data, n);
    if (w < 0) {
      if (errno == EINTR) continue;
      std::cerr << "write failed: " << std::strerror(errno) << "\n";
      return false;
    }
    data += static_cast<size_t>(w);
    n -= static_cast<size_t>(w);
  }
  return true;
}

// Read exactly n bytes unless timeout/EOF occurs. Returns how many bytes were read.
static size_t read_some(int fd, uint8_t* out, size_t n) {
  ssize_t r = ::read(fd, out, n);
  if (r < 0) {
    if (errno == EINTR) return 0;
    std::cerr << "read failed: " << std::strerror(errno) << "\n";
    return 0;
  }
  return static_cast<size_t>(r);
}

// Read exactly n bytes, looping until complete or a timeout yields 0 bytes.
static bool read_exact(int fd, uint8_t* out, size_t n) {
  size_t got = 0;
  while (got < n) {
    size_t r = read_some(fd, out + got, n - got);
    if (r == 0) return false; // timeout or EOF
    got += r;
  }
  return true;
}

// ------------------------- Packet logic -------------------------

static uint8_t compute_checksum(const std::vector<uint8_t>& pkt_without_checksum) {
  // Python: checksum = (256 - (sum(pkt[1:]) & 0xFF)) & 0xFF
  // pkt_without_checksum includes [FA, FF, MID, LEN, ...data...]
  uint32_t sum = 0;
  for (size_t i = 1; i < pkt_without_checksum.size(); i++) sum += pkt_without_checksum[i];
  uint8_t low = static_cast<uint8_t>(sum & 0xFF);
  uint8_t chk = static_cast<uint8_t>((256 - low) & 0xFF);
  return chk;
}

static bool send_packet(int fd, uint8_t mid, const std::vector<uint8_t>& data = {}) {
  std::vector<uint8_t> pkt;
  pkt.reserve(5 + data.size());

  pkt.push_back(0xFA);
  pkt.push_back(0xFF);
  pkt.push_back(mid);
  pkt.push_back(static_cast<uint8_t>(data.size()));
  pkt.insert(pkt.end(), data.begin(), data.end());

  uint8_t chk = compute_checksum(pkt);
  pkt.push_back(chk);

  if (!write_all(fd, pkt.data(), pkt.size())) return false;

  // python sleeps 0.1s after sending
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return true;
}

// ------------------------- Parsing helpers -------------------------

static uint16_t be_u16(const uint8_t* p) {
  return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

static float be_f32(const uint8_t* p) {
  // Convert big-endian 4 bytes -> float
  uint32_t u = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

struct Vec3 {
  float x{}, y{}, z{};
};

static std::optional<Vec3> parse_vec3_f32_be(const std::vector<uint8_t>& data) {
  if (data.size() != 12) return std::nullopt;
  Vec3 v;
  v.x = be_f32(&data[0]);
  v.y = be_f32(&data[4]);
  v.z = be_f32(&data[8]);
  return v;
}

// Verify checksum the same way the sender computes it:
// sum(bytes[1..end]) including checksum should be 0 mod 256.
static bool verify_checksum(uint8_t busid, uint8_t mid, uint8_t len,
                            const std::vector<uint8_t>& payload, uint8_t chk) {
  uint32_t sum = 0;
  sum += busid;
  sum += mid;
  sum += len;
  for (uint8_t b : payload) sum += b;
  sum += chk;
  return (sum & 0xFF) == 0;
}

// ------------------------- Main -------------------------

int main() {
  int fd = open_serial(PORT, BAUD);
  if (fd < 0) return 1;

  // 1) Go to config mode (MID 0x30)
  if (!send_packet(fd, 0x30)) return 1;

  // 2) Set output configuration payload:
  //    [0x40 0x20 0x00 0x64] [0x80 0x20 0x00 0x64]
  //    DID=0x4020 @100Hz, DID=0x8020 @100Hz
  std::vector<uint8_t> cfg = {0x40,0x20,0x00,0x64, 0x80,0x20,0x00,0x64};
  if (!send_packet(fd, 0xC0, cfg)) return 1;

  // 3) Go to measurement mode (MID 0x10)
  if (!send_packet(fd, 0x10)) return 1;

  std::cout << "Streaming Accelerometer (m/s^2) and Gyroscope (rad/s)...\n";

  while (true) {
    // Scan for header FA FF
    uint8_t b = 0;
    if (!read_exact(fd, &b, 1)) continue;
    if (b != 0xFA) continue;

    if (!read_exact(fd, &b, 1)) continue;
    if (b != 0xFF) continue;
    uint8_t busid = b;

    uint8_t mid = 0, len = 0;
    if (!read_exact(fd, &mid, 1)) continue;
    if (!read_exact(fd, &len, 1)) continue;

    std::vector<uint8_t> payload(len);
    if (len > 0) {
      if (!read_exact(fd, payload.data(), len)) continue;
    }

    uint8_t chk = 0;
    if (!read_exact(fd, &chk, 1)) continue;

    // Optional but recommended: verify checksum
    if (!verify_checksum(busid, mid, len, payload, chk)) {
      continue; // bad packet, resync
    }

    // Only accept MTData2 (MID 0x36)
    if (mid != 0x36) continue;

    std::optional<Vec3> acc;
    std::optional<Vec3> gyr;

    // Parse TLV blocks: [DID(2)][SIZE(1)][DATA(SIZE)]
    size_t i = 0;
    while (i + 3 <= payload.size()) {
      uint16_t did = be_u16(&payload[i]);
      uint8_t size = payload[i + 2];
      size_t start = i + 3;
      size_t end = start + size;
      if (end > payload.size()) break;

      std::vector<uint8_t> data(payload.begin() + start, payload.begin() + end);

      if (did == 0x4020) {          // Acceleration Float32 (3x float)
        acc = parse_vec3_f32_be(data);
      } else if (did == 0x8020) {   // RateOfTurn Float32 (3x float)
        gyr = parse_vec3_f32_be(data);
      }

      i = end;
    }

    // Print what we got
    if (acc || gyr) {
      std::cout << std::fixed << std::setprecision(3);

      bool first = true;
      if (acc) {
        std::cout << "ACC: X:" << std::setw(7) << acc->x
                  << " Y:" << std::setw(7) << acc->y
                  << " Z:" << std::setw(7) << acc->z << " m/s^2";
        first = false;
      }
      if (gyr) {
        if (!first) std::cout << " | ";
        std::cout << "GYR: X:" << std::setw(7) << gyr->x
                  << " Y:" << std::setw(7) << gyr->y
                  << " Z:" << std::setw(7) << gyr->z << " rad/s";
      }
      std::cout << "\n";
    }
  }

  ::close(fd);
  return 0;
}
