// imu_to_lcd.cpp
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <termios.h>
#include <chrono>

static constexpr const char* IMU_PORT = "/dev/ttymxc4";
static constexpr int IMU_BAUD = 115200;

static constexpr const char* LCD_PORT = "/dev/ttymxc2";
static constexpr int LCD_BAUD = 57600;

static constexpr uint8_t LCD_CMD = 0xFE;
static constexpr uint8_t LCD_LINES[4] = {0x00, 0x40, 0x14, 0x54};

static void die(const std::string& msg) {
    std::cerr << msg << " (errno=" << errno << "): " << std::strerror(errno) << "\n";
    std::exit(1);
}

static speed_t baudToTermios(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            std::cerr << "Unsupported baud: " << baud << "\n";
            std::exit(1);
    }
}

static int openSerial(const char* dev, int baud) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) die(std::string("Failed to open ") + dev);

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) die("tcgetattr failed");

    cfsetospeed(&tty, baudToTermios(baud));
    cfsetispeed(&tty, baudToTermios(baud));

    // 8N1, no flow control, raw mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit chars
    tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem controls, enable read
    tty.c_cflag &= ~(PARENB | PARODD);           // no parity
    tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                     // no HW flow control

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                     INLCR | IGNCR | ICRNL | IXON); // no SW flow control
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); // raw
    tty.c_oflag &= ~OPOST; // raw output

    // Read timeout: VTIME in deciseconds. Same spirit as Python timeout=1.
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10; // 1.0s

    if (tcsetattr(fd, TCSANOW, &tty) != 0) die("tcsetattr failed");
    return fd;
}

static bool writeAll(int fd, const uint8_t* data, size_t n) {
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::write(fd, data + off, n - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        off += static_cast<size_t>(w);
    }
    return true;
}

// Reads exactly n bytes unless timeout/EOF. Returns true if got all n.
static bool readExact(int fd, uint8_t* out, size_t n) {
    size_t off = 0;
    while (off < n) {
        ssize_t r = ::read(fd, out + off, n - off);
        if (r < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (r == 0) { // timeout (VMIN=0, VTIME>0) or EOF
            return false;
        }
        off += static_cast<size_t>(r);
    }
    return true;
}

static void sleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ---------- LCD helpers ----------
static void lcd_cmd(int lcdFd, uint8_t c) {
    uint8_t buf[2] = {LCD_CMD, c};
    writeAll(lcdFd, buf, sizeof(buf));
    sleepMs(2);
}

static void lcd_goto(int lcdFd, uint8_t addr) {
    uint8_t buf[3] = {LCD_CMD, 0x45, addr};
    writeAll(lcdFd, buf, sizeof(buf));
    sleepMs(2);
}

static void lcd_write_line(int lcdFd, const std::string& text) {
    std::string s = text;
    if (s.size() < 20) s.append(20 - s.size(), ' ');
    if (s.size() > 20) s.resize(20);

    writeAll(lcdFd, reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

static void lcd_clear_all(int lcdFd) {
    for (uint8_t a : LCD_LINES) {
        lcd_goto(lcdFd, a);
        lcd_write_line(lcdFd, "");
    }
}

// ---------- IMU packet send ----------
static void send_packet(int imuFd, uint8_t mid, const std::vector<uint8_t>& data = {}) {
    std::vector<uint8_t> pkt;
    pkt.reserve(5 + data.size());
    pkt.push_back(0xFA);
    pkt.push_back(0xFF);
    pkt.push_back(mid);
    pkt.push_back(static_cast<uint8_t>(data.size()));
    pkt.insert(pkt.end(), data.begin(), data.end());

    // checksum = (256 - (sum(pkt[1:]) & 0xFF)) & 0xFF
    uint32_t sum = 0;
    for (size_t i = 1; i < pkt.size(); i++) sum += pkt[i];
    uint8_t checksum = static_cast<uint8_t>((256 - (sum & 0xFF)) & 0xFF);
    pkt.push_back(checksum);

    writeAll(imuFd, pkt.data(), pkt.size());
    sleepMs(50);
}

// Big-endian bytes -> float (IEEE-754)
static float beBytesToFloat(const uint8_t b[4]) {
    uint32_t u = (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16) | (uint32_t(b[2]) << 8) | uint32_t(b[3]);
    float f;
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

struct Vec3 { float x{}, y{}, z{}; };

int main() {
    int imuFd = openSerial(IMU_PORT, IMU_BAUD);
    int lcdFd = openSerial(LCD_PORT, LCD_BAUD);

    // IMU init (same as Python)
    send_packet(imuFd, 0x30); // config mode

    // cfg = 40 20 00 64  80 20 00 64
    std::vector<uint8_t> cfg = {0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64};
    send_packet(imuFd, 0xC0, cfg);

    send_packet(imuFd, 0x10); // measurement mode

    // LCD init (same as Python)
    lcd_cmd(lcdFd, 0x41);
    sleepMs(100);
    lcd_clear_all(lcdFd);

    std::cout << "Streaming IMU data to LCD...\n";

    using clock = std::chrono::steady_clock;
    auto lastLcd = clock::now() - std::chrono::seconds(1);
    const auto lcdPeriod = std::chrono::milliseconds(200); // 5 Hz

    while (true) {
        // Find header FA FF
        uint8_t b = 0;
        if (!readExact(imuFd, &b, 1)) continue;
        if (b != 0xFA) continue;
        if (!readExact(imuFd, &b, 1)) continue;
        if (b != 0xFF) continue;

        uint8_t mid = 0, length = 0;
        if (!readExact(imuFd, &mid, 1)) continue;
        if (!readExact(imuFd, &length, 1)) continue;

        std::vector<uint8_t> payload(length);
        if (length > 0 && !readExact(imuFd, payload.data(), payload.size())) continue;

        uint8_t checksum = 0;
        if (!readExact(imuFd, &checksum, 1)) continue;
        // Note: like the Python version, we are NOT validating checksum here.

        if (mid != 0x36) continue;

        std::optional<Vec3> acc;
        std::optional<Vec3> gyr;

        // Parse blocks: [DID_hi, DID_lo, size, data...]
        size_t i = 0;
        while (i + 3 <= payload.size()) {
            uint16_t did = (uint16_t(payload[i]) << 8) | uint16_t(payload[i + 1]);
            uint8_t size = payload[i + 2];
            size_t dataStart = i + 3;
            size_t dataEnd = dataStart + size;
            if (dataEnd > payload.size()) break;

            const uint8_t* data = payload.data() + dataStart;

            if (did == 0x4020 && size == 12) {
                Vec3 v;
                v.x = beBytesToFloat(data + 0);
                v.y = beBytesToFloat(data + 4);
                v.z = beBytesToFloat(data + 8);
                acc = v;
            } else if (did == 0x8020 && size == 12) {
                Vec3 v;
                v.x = beBytesToFloat(data + 0);
                v.y = beBytesToFloat(data + 4);
                v.z = beBytesToFloat(data + 8);
                gyr = v;
            }

            i += 3 + size;
        }

        if (!acc.has_value() || !gyr.has_value()) continue;

        auto now = clock::now();
        if (now - lastLcd < lcdPeriod) continue;
        lastLcd = now;

        char line[64];

        lcd_goto(lcdFd, 0x00);
        std::snprintf(line, sizeof(line), "ACC X:%5.2f Y:%5.2f", acc->x, acc->y);
        lcd_write_line(lcdFd, line);

        lcd_goto(lcdFd, 0x40);
        std::snprintf(line, sizeof(line), "ACC Z:%5.2f m/s2", acc->z);
        lcd_write_line(lcdFd, line);

        lcd_goto(lcdFd, 0x14);
        std::snprintf(line, sizeof(line), "GYR X:%5.2f Y:%5.2f", gyr->x, gyr->y);
        lcd_write_line(lcdFd, line);

        lcd_goto(lcdFd, 0x54);
        std::snprintf(line, sizeof(line), "GYR Z:%5.2f rad/s", gyr->z);
        lcd_write_line(lcdFd, line);
    }

    ::close(imuFd);
    ::close(lcdFd);
    return 0;
}
