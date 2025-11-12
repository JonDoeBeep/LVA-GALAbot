#include "Netman.h"
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <iostream> 
#include <frc/Timer.h>
#include <algorithm>
#include <cstring>

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

void Netman::ProcessPacket(const uint8_t* data, size_t length) {
    // new fancy packet format:
    // header: [version(1)][flags(1)][command_count(2)]
    // each command: [vx(4)][omega(4)][duration_ms(2)]
    
    if (length < 4) return; // need at least header
    
    uint8_t version = data[0];
    uint8_t flags = data[1];
    uint16_t command_count = (data[2] << 8) | data[3];
    
    // version check
    if (version != 1) {
        std::cerr << "Unsupported packet version: " << (int)version << std::endl;
        return;
    }
    
    if (flags & 0x01) {
        ClearQueue();
    }
    
    size_t offset = 4;
    constexpr size_t kCommandSize = 10; // 4 + 4 + 2 bytes
    
    for (uint16_t i = 0; i < command_count && offset + kCommandSize <= length; ++i) {
        float vx_raw, omega_raw;
        uint16_t duration_ms;
        
        // parse command (little endian)
        std::memcpy(&vx_raw, &data[offset], 4);
        std::memcpy(&omega_raw, &data[offset + 4], 4);
        duration_ms = (data[offset + 8] << 8) | data[offset + 9];
        
        Command cmd;
        cmd.vx = units::meters_per_second_t{
            std::clamp((double)vx_raw, -DriveConstants::kMaxSpeed.value(), DriveConstants::kMaxSpeed.value())
        };
        cmd.omega = units::radians_per_second_t{
            std::clamp((double)omega_raw, -DriveConstants::kMaxAngularSpeed.value(), DriveConstants::kMaxAngularSpeed.value())
        };
        cmd.duration = units::second_t{duration_ms / 1000.0};
        
        m_commandQueue.push(cmd);
        offset += kCommandSize;
    }
    
    m_lastRxTime = frc::Timer::GetFPGATimestamp();
}

void Netman::Periodic() {
    if (m_udpSock < 0) return;

    // big packet upgrade
    uint8_t buffer[1024]; // support up to 100ish commands per usp pacet
    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    int n = recvfrom(m_udpSock, buffer, sizeof(buffer), MSG_DONTWAIT, (sockaddr*)&src, &slen);
    
    if (n > 0) {
        ProcessPacket(buffer, n);
    }
    
    auto now = frc::Timer::GetFPGATimestamp();
    
    // check if current command expired
    if (m_commandStartTime != 0_s && 
        m_currentCommand.duration > 0_s &&
        (now - m_commandStartTime) >= m_currentCommand.duration) {
        m_commandStartTime = 0_s;
    }
    
    // load next command
    if (m_commandStartTime == 0_s && !m_commandQueue.empty()) {
        m_currentCommand = m_commandQueue.front();
        m_commandQueue.pop();
        m_commandStartTime = now;
    }
}

std::optional<Netman::Command> Netman::GetCommand() {
    if (m_lastRxTime == 0_s) {
        return std::nullopt;
    }
    
    // if active command, return it
    if (m_commandStartTime != 0_s) {
        return m_currentCommand;
    }
    
    return std::nullopt;
}

bool Netman::IsConnected() {
    return (frc::Timer::GetFPGATimestamp() - m_lastRxTime) < NetworkConstants::kCommandTimeout;
}

void Netman::ClearQueue() {
    while (!m_commandQueue.empty()) {
        m_commandQueue.pop();
    }
    m_commandStartTime = 0_s;
}