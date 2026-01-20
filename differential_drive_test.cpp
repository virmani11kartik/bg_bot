// differential_drive_test.cpp
// C++ version of the Python script: send RPDO to node 1 & 2, then SYNC (CANopen-style)
// Build:  g++ -O2 -std=c++17 differential_drive_test.cpp -o differential_drive_test
// Run:    sudo ./differential_drive_test.cpp
//


#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

static constexpr double CMD_PER_WHEEL_RPM = 3814.0;

class CanSocket {
public:
    explicit CanSocket(const std::string& ifname) {
        sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            perror("socket(PF_CAN)");
            throw std::runtime_error("Failed to create CAN socket");
        }

        struct ifreq ifr {};
        std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl(SIOCGIFINDEX)");
            ::close(sock_);
            throw std::runtime_error("Failed to get interface index (is can0 up?)");
        }

        struct sockaddr_can addr {};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("bind(AF_CAN)");
            ::close(sock_);
            throw std::runtime_error("Failed to bind CAN socket");
        }
    }

    ~CanSocket() {
        if (sock_ >= 0) ::close(sock_);
    }

    void sendFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
        struct can_frame frame {};
        frame.can_id  = can_id; // standard 11-bit ID (no CAN_EFF_FLAG)
        frame.can_dlc = dlc;

        if (dlc > 0 && data != nullptr) {
            std::memcpy(frame.data, data, dlc);
        }

        const ssize_t n = ::write(sock_, &frame, sizeof(frame));
        if (n != static_cast<ssize_t>(sizeof(frame))) {
            perror("write(can_frame)");
            throw std::runtime_error("Failed to write CAN frame");
        }
    }

private:
    int sock_{-1};
};

// Pack little-endian: <HiH  (uint16 ctrl, int32 vel, uint16 0) into 8 bytes
static inline void packControlVel(uint16_t ctrl, int32_t vel, uint8_t out[8]) {
    out[0] = static_cast<uint8_t>( ctrl        & 0xFF);
    out[1] = static_cast<uint8_t>((ctrl >> 8)  & 0xFF);

    out[2] = static_cast<uint8_t>( vel        & 0xFF);
    out[3] = static_cast<uint8_t>((vel >> 8)  & 0xFF);
    out[4] = static_cast<uint8_t>((vel >> 16) & 0xFF);
    out[5] = static_cast<uint8_t>((vel >> 24) & 0xFF);

    out[6] = 0;
    out[7] = 0;
}

static inline void sleepSeconds(double s) {
    using namespace std::chrono;
    std::this_thread::sleep_for(duration_cast<nanoseconds>(duration<double>(s)));
}

// Send RPDO command (0x500 + node) to one motor
static void sendCmdMotor(CanSocket& can, uint8_t node, uint16_t ctrl, int32_t vel = 0) {
    const uint32_t rpdo_id = 0x500u + node; // node 1 -> 0x501, node 2 -> 0x502
    uint8_t payload[8];
    packControlVel(ctrl, vel, payload);

    can.sendFrame(rpdo_id, payload, 8);
    sleepSeconds(0.001); // ~1ms like Python
}

// Send SYNC (0x080)
static void sendSync(CanSocket& can) {
    can.sendFrame(0x080u, nullptr, 0);
    sleepSeconds(0.008); // ~8ms like Python
}

// Set both wheel speeds and then send a single SYNC
static void setVelocities(CanSocket& can, double left_rpm, double right_rpm) {
    const int32_t left_cmd  = static_cast<int32_t>(std::llround(left_rpm  * CMD_PER_WHEEL_RPM));
    const int32_t right_cmd = static_cast<int32_t>(std::llround(right_rpm * CMD_PER_WHEEL_RPM));

    // ctrl 0x000F matches the Python script
    sendCmdMotor(can, 1, 0x000F, left_cmd);   // Left motor (node 1)
    sendCmdMotor(can, 2, 0x000F, right_cmd);  // Right motor (node 2)

    // One SYNC applies both
    sendSync(can);
}

struct Movement {
    double left_rpm;
    double right_rpm;
    double duration_s;
    const char* desc;
};

int main() {
    try {
        CanSocket can("can0");

        std::cout << std::string(70, '=') << "\n";
        std::cout << "DIFFERENTIAL DRIVE ROBOT CONTROL\n";
        std::cout << std::string(70, '=') << "\n";

        std::vector<Movement> movements = {
            {  50, -50, 2, "Forward"},
            {   0,   0, 1, "Stop"},
            { -50,  50, 2, "Backward"},
            {   0,   0, 1, "Stop"},
            {  50,  50, 2, "Rotate Right"},
            {   0,   0, 1, "Stop"},
            { -50, -50, 2, "Rotate Left"},
            {   0,   0, 1, "Stop"},
        };

        std::cout << "\nExecuting movement sequence...\n\n";

        for (const auto& m : movements) {
            std::cout << "→ " << m.desc << ": L=" << m.left_rpm
                      << " R=" << m.right_rpm << " RPM\n";

            const auto t0 = std::chrono::steady_clock::now();
            while (true) {
                const auto now = std::chrono::steady_clock::now();
                const double elapsed =
                    std::chrono::duration<double>(now - t0).count();
                if (elapsed >= m.duration_s) break;

                setVelocities(can, m.left_rpm, m.right_rpm);
            }
            std::cout << "  Complete\n\n";
        }

        // Stop both motors
        std::cout << "Stopping both motors...\n";
        for (int i = 0; i < 30; ++i) {
            setVelocities(can, 0.0, 0.0);
        }

        // Disable both motors (ctrl 0x0006 like Python)
        for (int i = 0; i < 10; ++i) {
            sendCmdMotor(can, 1, 0x0006, 0);
            sendCmdMotor(can, 2, 0x0006, 0);
            sendSync(can);
        }

        std::cout << "\n✓ Robot control test complete!\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
