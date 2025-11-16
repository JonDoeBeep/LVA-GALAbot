#include "Netman.h"
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <iostream> 
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <cstring>
#include <cerrno>

Netman::Netman() {
    m_udpSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_udpSock < 0) {
        std::cerr << "CRITICAL ERROR: Failed to create UDP socket! errno=" << errno << std::endl;
        return;
    }

    int yes = 1;
    if (setsockopt(m_udpSock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
        std::cerr << "WARNING: Failed to set SO_REUSEADDR. errno=" << errno << std::endl;
    }
    
    if (fcntl(m_udpSock, F_SETFL, O_NONBLOCK) < 0) {
        std::cerr << "CRITICAL ERROR: Failed to set nonblocking mode! errno=" << errno << std::endl;
        close(m_udpSock);
        m_udpSock = -1;
        return;
    }

    m_udpAddr = {};
    m_udpAddr.sin_family = AF_INET;
    m_udpAddr.sin_port = htons(NetworkConstants::kUdpPort);
    m_udpAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(m_udpSock, (sockaddr*)&m_udpAddr, sizeof(m_udpAddr)) < 0) {
        std::cerr << "CRITICAL ERROR: Failed to bind UDP socket on port " 
                  << NetworkConstants::kUdpPort << "! errno=" << errno << std::endl;
        close(m_udpSock);
        m_udpSock = -1;
    }
}

Netman::~Netman() {
    if (m_udpSock >= 0) {
        close(m_udpSock);
    }
}

bool Netman::ProcessPacket(const uint8_t* data, size_t length) {
    // Packet format (big endian):
    // Header: [major(1)][minor(1)][flags(1)][command_count(1)]
    // Each command: [vx][omega][duration_ms]
    
    if (length < NetworkConstants::kHeaderSize) {
        std::cerr << "ERROR: Packet too small (" << length << " bytes)" << std::endl;
        m_parseErrors++;
        return false;
    }
    
    uint8_t major = data[0];
    uint8_t minor = data[1];
    uint8_t flags = data[2];
    uint8_t command_count = data[3];
    
    // version check - must match major.minor (ignore patch)
    if (major != NetworkConstants::kProtocolMajor || minor != NetworkConstants::kProtocolMinor) {
        std::cerr << "ERROR: Unsupported protocol version: " << int(major) << "." << int(minor)
                  << " (expected " << int(NetworkConstants::kProtocolMajor) << "." 
                  << int(NetworkConstants::kProtocolMinor) << ")" << std::endl;
        m_parseErrors++;
        return false;
    }
    
    // sanity check command count
    if (command_count > NetworkConstants::kMaxCommandsPerPacket) {
        std::cerr << "ERROR: Too many commands: " << int(command_count)
                  << " (max " << NetworkConstants::kMaxCommandsPerPacket << ")" << std::endl;
        m_parseErrors++;
        return false;
    }
    
    // validate packet size
    size_t expected_size = NetworkConstants::kHeaderSize + 
                          (command_count * NetworkConstants::kCommandSize);
    if (length < expected_size) {
        std::cerr << "ERROR: Packet truncated. Expected " << expected_size 
                  << " bytes, got " << length << std::endl;
        m_parseErrors++;
        return false;
    }
    
    // clear queue flag
    if (flags & NetworkConstants::kFlagClearQueue) {
        ClearQueue();
    }
    
    size_t offset = NetworkConstants::kHeaderSize;
    
    for (uint16_t i = 0; i < command_count; ++i) {
        // check queue size limit to prevent DOS
        if (m_commandQueue.size() >= NetworkConstants::kMaxQueueSize) {
            std::cerr << "WARNING: Command queue full! Dropping remaining commands." << std::endl;
            m_queueOverflows++;
            break;
        }
        
        // parse command using memcpy to avoid strict aliasing violation
        uint32_t vx_bits, omega_bits;
        uint16_t duration_ms;
        
        // extract big-endian values
        vx_bits = (uint32_t(data[offset]) << 24) | 
                  (uint32_t(data[offset + 1]) << 16) |
                  (uint32_t(data[offset + 2]) << 8) | 
                  uint32_t(data[offset + 3]);
        
        omega_bits = (uint32_t(data[offset + 4]) << 24) | 
                     (uint32_t(data[offset + 5]) << 16) |
                     (uint32_t(data[offset + 6]) << 8) | 
                     uint32_t(data[offset + 7]);
        
        duration_ms = (uint16_t(data[offset + 8]) << 8) | uint16_t(data[offset + 9]);
        
        // reinterpret bits as floats using memcpy (safe aliasing)
        float vx_raw, omega_raw;
        std::memcpy(&vx_raw, &vx_bits, sizeof(float));
        std::memcpy(&omega_raw, &omega_bits, sizeof(float));
        
        // validate floats (check for NaN/Inf)
        if (!std::isfinite(vx_raw) || !std::isfinite(omega_raw)) {
            std::cerr << "ERROR: Invalid float values in command " << i << std::endl;
            m_parseErrors++;
            continue; // skip this command but process others
        }
        
        Command cmd;
        cmd.vx = units::meters_per_second_t{
            std::clamp(double(vx_raw), 
                      -DriveConstants::kMaxSpeed.value(), 
                      DriveConstants::kMaxSpeed.value())
        };
        cmd.omega = units::radians_per_second_t{
            std::clamp(double(omega_raw), 
                      -DriveConstants::kMaxAngularSpeed.value(), 
                      DriveConstants::kMaxAngularSpeed.value())
        };
        cmd.duration = units::second_t{duration_ms / 1000.0};
        
        m_commandQueue.push(cmd);
        offset += NetworkConstants::kCommandSize;
    }
    
    m_lastRxTime = frc::Timer::GetFPGATimestamp();
    return true;
}

void Netman::Periodic() {
    if (m_udpSock < 0) return;

    uint8_t buffer[NetworkConstants::kMaxPacketSize];
    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    
    ssize_t n = recvfrom(m_udpSock, buffer, sizeof(buffer), MSG_DONTWAIT, 
                        (sockaddr*)&src, &slen);
    
    if (n > 0) {
        m_packetsReceived++;
        ProcessPacket(buffer, static_cast<size_t>(n));
    } else if (n < 0) {
        // handle recv errors properly
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            // real error occurred
            std::cerr << "ERROR: recvfrom failed with errno=" << errno << std::endl;
            
            // check for fatal errors
            if (errno == EBADF || errno == ENOTSOCK) {
                std::cerr << "FATAL: Socket is invalid, cannot recover" << std::endl;
                // socket is bad, close it
                close(m_udpSock);
                m_udpSock = -1;
            }
        }
        // EAGAIN/EWOULDBLOCK is normal for non-blocking socket with no data
    }

    auto now = frc::Timer::GetFPGATimestamp();

    // expire current command
    if (m_commandStartTime != 0_s &&
        m_currentCommand.duration > 0_s &&
        (now - m_commandStartTime) >= m_currentCommand.duration) {
        m_commandStartTime = 0_s;
    }

    // load next
    if (m_commandStartTime == 0_s && !m_commandQueue.empty()) {
        m_currentCommand = m_commandQueue.front();
        m_commandQueue.pop();
        m_commandStartTime = now;
    }
    
    // publish stats to SmartDashboard
    frc::SmartDashboard::PutNumber("Network/PacketsRx", m_packetsReceived);
    frc::SmartDashboard::PutNumber("Network/ParseErrors", m_parseErrors);
    frc::SmartDashboard::PutNumber("Network/QueueOverflows", m_queueOverflows);
}

std::optional<Netman::Command> Netman::GetCommand() {
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