#include "Netman.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <chrono>

Netman::Netman() 
    : m_spi(frc::SPI::Port::kOnboardCS0)
{
    // Configure SPI
    m_spi.SetClockRate(1000000); // 1 MHz
    m_spi.SetMode(frc::SPI::Mode::kMode0);
    m_spi.SetChipSelectActiveLow();

    // Configure BCNP Controller
    bcnp::ControllerConfig config;
    config.limits.vxMax = DriveConstants::kMaxSpeed.value();
    config.limits.vxMin = -DriveConstants::kMaxSpeed.value();
    config.limits.omegaMax = DriveConstants::kMaxAngularSpeed.value();
    config.limits.omegaMin = -DriveConstants::kMaxAngularSpeed.value();
    
    m_controller = bcnp::Controller(config);
}

Netman::~Netman() {
}

void Netman::Periodic() {
    // Read from SPI
    uint8_t buffer[1024];
    int bytesRead = m_spi.Read(true, buffer, sizeof(buffer));
    
    if (bytesRead > 0) {
        m_controller.PushBytes(buffer, bytesRead);
    }

    // Update stats
    auto& metrics = m_controller.Queue().Metrics();
    frc::SmartDashboard::PutNumber("Network/PacketsRx", static_cast<double>(metrics.packetsReceived));
    frc::SmartDashboard::PutNumber("Network/ParseErrors", static_cast<double>(metrics.parseErrors));
    frc::SmartDashboard::PutNumber("Network/QueueOverflows", static_cast<double>(metrics.queueOverflows));
}

std::optional<Netman::Command> Netman::GetCommand() {
    auto now = std::chrono::steady_clock::now();
    auto bcnpCmd = m_controller.CurrentCommand(now);
    
    if (bcnpCmd) {
        Netman::Command cmd;
        cmd.vx = units::meters_per_second_t{bcnpCmd->vx};
        cmd.omega = units::radians_per_second_t{bcnpCmd->omega};
        cmd.duration = units::millisecond_t{bcnpCmd->durationMs};
        return cmd;
    }
    return std::nullopt;
}

bool Netman::IsConnected() {
    return m_controller.IsConnected(std::chrono::steady_clock::now());
}

void Netman::ClearQueue() {
    m_controller.Queue().Clear();
}