#pragma once

#include <array>
#include <cstdint>
#include <stdexcept>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rmcs_mujoco_bridge {

using JointValues = std::array<double, 7>;

struct SimulateState {
    double time;
    JointValues position, velocity, torque, target;
    std::array<double, 3> gravity;
    double gripper_angle, gripper_velocity, gripper_torque, gripper_target;
};

struct SimulateTorque {
    JointValues arm;
    double gripper;
};

inline constexpr std::uint16_t kRmcsPort = 17971;
inline constexpr std::uint16_t kSimulatePort = 17972;

class SimulateLink {
public:
    SimulateLink(std::uint16_t local_port, std::uint16_t peer_port)
        : fd_(socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)) {
        if (fd_ < 0)
            throw std::runtime_error("Could not open the MuJoCo loopback socket");
        const auto address = [](std::uint16_t port) {
            sockaddr_in result{};
            result.sin_family = AF_INET;
            result.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            result.sin_port = htons(port);
            return result;
        };
        const auto local = address(local_port);
        const auto peer = address(peer_port);
        if (bind(fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0
            || connect(fd_, reinterpret_cast<const sockaddr*>(&peer), sizeof(peer)) != 0) {
            close(fd_);
            throw std::runtime_error("Could not bind or connect the MuJoCo loopback socket");
        }
    }

    ~SimulateLink() { close(fd_); }
    SimulateLink(const SimulateLink&) = delete;
    SimulateLink& operator=(const SimulateLink&) = delete;

    template <typename Packet>
    bool ReceiveLatest(Packet& packet) const {
        bool received = false;
        for (int i = 0; i < 32; ++i) {
            Packet sample;
            const auto size = recv(fd_, &sample, sizeof(sample), MSG_TRUNC);
            if (size < 0)
                break;
            if (static_cast<std::size_t>(size) == sizeof(sample)) {
                packet = sample;
                received = true;
            }
        }
        return received;
    }

    template <typename Packet>
    void Send(const Packet& packet) const {
        (void)send(fd_, &packet, sizeof(packet), 0);
    }

private:
    int fd_;
};

} // namespace rmcs_mujoco_bridge
