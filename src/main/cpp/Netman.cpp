#include "Netman.h"
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <iostream> 
#include <frc/Timer.h>
#include <algorithm>

Netman::Netman() {
    m_udpSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSock < 0) {
        std::cerr << "CRITICAL ERROR: Failed to create UDP socket!" << std::endl;
        return;
    }

    int yes = 1;
    setsockopt(m_udpSock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    fcntl(m_udpSock, F_SETFL, O_NONBLOCK);

    m_udpAddr = {};
    m_udpAddr.sin_family = AF_INET;
    m_udpAddr.sin_port = htons(NetworkConstants::kUdpPort);
    m_udpAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(m_udpSock, (sockaddr*)&m_udpAddr, sizeof(m_udpAddr)) < 0) {
        std::cerr << "CRITICAL ERROR: Failed to bind UDP socket on port " << NetworkConstants::kUdpPort << "!" << std::endl;
        close(m_udpSock);
        m_udpSock = -1;
    }
}

void Netman::Periodic() {
    if (m_udpSock < 0) return;

    struct __attribute__((packed)) Pkt {
        int64_t ts_ns;
        float   vx;
        float   omega;
        uint16_t ttl_ms;
        uint8_t  mode_hint;
        uint8_t  pad;
    } pkt;

    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    int n = recvfrom(m_udpSock, &pkt, sizeof(pkt), MSG_DONTWAIT, (sockaddr*)&src, &slen);
    
    if (n == (int)sizeof(pkt)) {
        m_lastRxTime = frc::Timer::GetFPGATimestamp();
        
        m_lastCommand.vx = units::meters_per_second_t{
            std::clamp((double)pkt.vx, -DriveConstants::kMaxSpeed.value(), DriveConstants::kMaxSpeed.value())
        };
        m_lastCommand.omega = units::radians_per_second_t{
            std::clamp((double)pkt.omega, -DriveConstants::kMaxAngularSpeed.value(), DriveConstants::kMaxAngularSpeed.value())
        };
    }
}

std::optional<Netman::Command> Netman::GetCommand() {
    if (m_lastRxTime == 0_s) {
        return std::nullopt;
    }
    return m_lastCommand;
}

bool Netman::IsConnected() {
    return (frc::Timer::GetFPGATimestamp() - m_lastRxTime) < NetworkConstants::kCommandTimeout;
}

// well commented code.