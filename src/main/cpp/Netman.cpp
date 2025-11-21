#include "Netman.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <chrono>

static bcnp::ControllerConfig MakeControllerConfig() {
    bcnp::ControllerConfig config;
    config.limits.vxMax = DriveConstants::kMaxSpeed.value();
    config.limits.vxMin = -DriveConstants::kMaxSpeed.value();
    config.limits.omegaMax = DriveConstants::kMaxAngularSpeed.value();
    config.limits.omegaMin = -DriveConstants::kMaxAngularSpeed.value();
    config.limits.durationMin = 0;
    config.limits.durationMax = 65535; // Max uint16_t
    
    // Configure queue timeouts
    config.queue.connectionTimeout = std::chrono::milliseconds(200);
    config.queue.maxCommandLag = std::chrono::milliseconds(5000); // Allow 5s lag for batched commands
    config.queue.capacity = bcnp::kMaxCommandsPerPacket; // Allow full packet of commands
    config.parserBufferSize = bcnp::kMaxPacketSize; // Buffer large enough for max packet
    
    return config;
}

Netman::Netman() 
    : m_controller(MakeControllerConfig())
{
    // Create TCP adapter in server mode (listen for incoming connections)
    m_tcpAdapter = std::make_unique<bcnp::TcpPosixAdapter>(kTcpPort);
    
    // Create controller driver to connect adapter to controller
    m_driver = std::make_unique<bcnp::ControllerDriver>(m_controller, *m_tcpAdapter);
}

Netman::~Netman() {
}

void Netman::Periodic() {
    // Poll driver to receive data from TCP adapter and feed to controller
    m_driver->PollOnce();
    
    // Update controller state with current time
    auto now = std::chrono::steady_clock::now();
    m_controller.Queue().Update(now);

    // Update stats
    auto metrics = m_controller.Queue().GetMetrics();
    frc::SmartDashboard::PutNumber("Network/PacketsRx", static_cast<double>(metrics.packetsReceived));
    frc::SmartDashboard::PutNumber("Network/ParseErrors", static_cast<double>(metrics.parseErrors));
    frc::SmartDashboard::PutNumber("Network/QueueOverflows", static_cast<double>(metrics.queueOverflows));
    frc::SmartDashboard::PutBoolean("Network/Connected", IsConnected());
    frc::SmartDashboard::PutNumber("Network/QueueSize", static_cast<double>(GetQueueSize()));
    
    // Update current command info
    auto cmd = GetCommand();
    if (cmd) {
        frc::SmartDashboard::PutNumber("Network/CmdVx", cmd->vx.value());
        frc::SmartDashboard::PutNumber("Network/CmdW", cmd->omega.value());
    } else {
        frc::SmartDashboard::PutNumber("Network/CmdVx", 0.0);
        frc::SmartDashboard::PutNumber("Network/CmdW", 0.0);
    }
}

std::optional<Netman::Command> Netman::GetCommand() {
    // Get active command from queue
    auto bcnpCmd = m_controller.Queue().ActiveCommand();
    
    if (bcnpCmd) {
        Netman::Command cmd;
        cmd.vx = units::meters_per_second_t{bcnpCmd->vx};
        cmd.omega = units::radians_per_second_t{bcnpCmd->omega};
        cmd.duration = units::millisecond_t{static_cast<double>(bcnpCmd->durationMs)};
        return cmd;
    }
    return std::nullopt;
}

bool Netman::IsConnected() {
    auto now = std::chrono::steady_clock::now();
    return m_controller.Queue().IsConnected(now) && m_tcpAdapter->IsConnected();
}

void Netman::ClearQueue() {
    m_controller.Queue().Clear();
}