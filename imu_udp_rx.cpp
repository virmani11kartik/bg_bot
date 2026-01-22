// imu_udp_rx.cpp
#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>

static constexpr int LISTEN_PORT = 5005;

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
  uint32_t magic;        // "UIMU"
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
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) { std::perror("socket"); return 1; }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(LISTEN_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    std::perror("bind");
    return 1;
  }

  std::printf("Listening UDP on 0.0.0.0:%d\n", LISTEN_PORT);

  while (true) {
    UdpImuPacket pkt{};
    sockaddr_in src{};
    socklen_t slen = sizeof(src);

    ssize_t n = recvfrom(fd, &pkt, sizeof(pkt), 0, reinterpret_cast<sockaddr*>(&src), &slen);
    if (n < 0) { std::perror("recvfrom"); continue; }
    if (n != (ssize_t)sizeof(UdpImuPacket)) continue;

    if (pkt.magic != 0x554D4955u || pkt.version != 1 || pkt.payload_len != sizeof(UdpImuPacket)) continue;

    uint32_t rx_crc = pkt.crc;
    pkt.crc = 0;
    uint32_t calc = crc32(reinterpret_cast<const uint8_t*>(&pkt), sizeof(UdpImuPacket) - sizeof(uint32_t));
    if (calc != rx_crc) {
      std::fprintf(stderr, "CRC mismatch (seq=%u)\n", pkt.seq);
      continue;
    }

    char ip[INET_ADDRSTRLEN]{};
    inet_ntop(AF_INET, &src.sin_addr, ip, sizeof(ip));

    std::printf("src=%s seq=%u t=%lu roll=%+.3f pitch=%+.3f yaw=%+.3f\n",
                ip, pkt.seq, (unsigned long)pkt.t_monotonic_ns,
                pkt.roll, pkt.pitch, pkt.yaw);
    std::fflush(stdout);
  }

  ::close(fd);
  return 0;
}
