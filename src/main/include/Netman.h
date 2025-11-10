#pragma once

#include <netinet/in.h>
#include <units/time.h>
#include <optional>
#include "Constants.h"

// udp class
class Netman {
public:
    // hold command pkt
    struct Command {
        units::meters_per_second_t vx{0.0};
        units::radians_per_second_t omega{0.0};
    };

    Netman();
    
    // check new
    void Periodic();
    
    std::optional<Command> GetCommand();
    
    bool IsConnected();

private:
    int m_udpSock{-1};
    sockaddr_in m_udpAddr{};
    
    Command m_lastCommand;
    units::second_t m_lastRxTime{0_s};
};