#pragma once

#include <units/time.h>
#include <optional>
#include <vector>
#include <queue>
#include <cstdint>
#include <memory>
#include "bcnp/controller.h"
#include "bcnp/transport/tcp_posix.h"
#include "bcnp/transport/controller_driver.h"
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
    
    size_t GetQueueSize() const { return m_controller.Queue().Size(); }
    
    void ClearQueue();

private:
    static constexpr uint16_t kTcpPort = 5800; // BCNP TCP port
    
    bcnp::Controller m_controller;
    std::unique_ptr<bcnp::TcpPosixAdapter> m_tcpAdapter;
    std::unique_ptr<bcnp::ControllerDriver> m_driver;
};