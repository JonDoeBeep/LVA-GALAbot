#pragma once

#include <netinet/in.h>
#include <units/time.h>
#include <optional>
#include <vector>
#include <queue>
#include <cstdint>
#include "Constants.h"

class Netman {
public:
    // hold command pkt
    struct Command {
        units::meters_per_second_t vx{0.0};
        units::radians_per_second_t omega{0.0};
        units::second_t duration{0.0_s}; // how long to execute command
    };

    Netman();
    ~Netman();
    
    // check new
    void Periodic();
    
    std::optional<Command> GetCommand();
    
    bool IsConnected();
    
    size_t GetQueueSize() const { return m_commandQueue.size(); }
    
    void ClearQueue();

private:
    int m_udpSock{-1};
    sockaddr_in m_udpAddr{};
    
    Command m_currentCommand;
    units::second_t m_lastRxTime{0_s};
    
    // command queue for batch
    std::queue<Command> m_commandQueue;
    units::second_t m_commandStartTime{0_s};
    
    // process received and extract
    // ret false on parse error
    bool ProcessPacket(const uint8_t* data, size_t length);
    
    // stats
    uint32_t m_packetsReceived{0};
    uint32_t m_parseErrors{0};
    uint32_t m_queueOverflows{0};
};