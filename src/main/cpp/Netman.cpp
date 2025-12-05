#include "Netman.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <chrono>
#include <sstream>
#include <iomanip>

static bcnp::MessageQueueConfig MakeQueueConfig() {
    bcnp::MessageQueueConfig config;
    // Configure queue timeouts
    config.connectionTimeout = std::chrono::milliseconds(200);
    config.maxCommandLag = std::chrono::milliseconds(5000); // Allow 5s lag for batched commands
    config.capacity = 256; // Fixed capacity for real-time safety (v3.2 static vector compatible)
    return config;
}

static bcnp::DispatcherConfig MakeDispatcherConfig() {
    bcnp::DispatcherConfig config;
    config.parserBufferSize = 256 * 4096;
    config.connectionTimeout = std::chrono::milliseconds(200);
    return config;
}

static bcnp::TelemetryAccumulatorConfig MakeTelemetryConfig() {
    bcnp::TelemetryAccumulatorConfig config;
    config.flushIntervalTicks = 2;  // Send every 2 ticks (25Hz at 50Hz loop)
    config.maxBufferedMessages = 32;
    return config;
}

Netman::Netman() 
    : m_driveQueue(MakeQueueConfig()),
      m_dispatcher(MakeDispatcherConfig()),
      m_telemetryAccumulator(MakeTelemetryConfig())
{
    // Register DriveCmd handler to push commands into queue
    m_dispatcher.RegisterHandler<bcnp::DriveCmd>([this](const bcnp::PacketView& pkt) {
        // Check clear queue flag FIRST - critical for STOP commands
        if (pkt.header.flags & bcnp::kFlagClearQueue) {
            m_driveQueue.Clear();
        }
        
        for (auto it = pkt.begin_as<bcnp::DriveCmd>(); it != pkt.end_as<bcnp::DriveCmd>(); ++it) {
            // Clamp velocities to robot limits
            bcnp::DriveCmd cmd = *it;
            cmd.vx = std::clamp(cmd.vx, 
                -static_cast<float>(DriveConstants::kMaxSpeed.value()), 
                static_cast<float>(DriveConstants::kMaxSpeed.value()));
            cmd.omega = std::clamp(cmd.omega, 
                -static_cast<float>(DriveConstants::kMaxAngularSpeed.value()), 
                static_cast<float>(DriveConstants::kMaxAngularSpeed.value()));
            m_driveQueue.Push(cmd);
        }
        m_driveQueue.NotifyReceived(bcnp::MessageQueue<bcnp::DriveCmd>::Clock::now());
    });
    
    // Create TCP adapter in server mode (listen for incoming connections)
    m_tcpAdapter = std::make_unique<bcnp::TcpPosixAdapter>(kTcpPort);
    
    // Create dispatcher driver to connect adapter to dispatcher
    m_driver = std::make_unique<bcnp::DispatcherDriver>(m_dispatcher, *m_tcpAdapter);
}

Netman::~Netman() {
}

void Netman::Periodic() {
    // Poll driver to receive data from TCP adapter and feed to dispatcher
    m_driver->PollOnce();
    
    // Update queue state with current time
    auto now = std::chrono::steady_clock::now();
    m_driveQueue.Update(now);

    // Update stats
    auto metrics = m_driveQueue.GetMetrics();
    frc::SmartDashboard::PutNumber("Network/MessagesRx", static_cast<double>(metrics.messagesReceived));
    frc::SmartDashboard::PutNumber("Network/ParseErrors", static_cast<double>(m_dispatcher.ParseErrorCount()));
    frc::SmartDashboard::PutNumber("Network/QueueOverflows", static_cast<double>(metrics.queueOverflows));
    frc::SmartDashboard::PutBoolean("Network/Connected", IsConnected());
    frc::SmartDashboard::PutNumber("Network/QueueSize", static_cast<double>(GetQueueSize()));
    
    // Publish schema hash for diagnostics
    std::ostringstream hashStr;
    hashStr << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << bcnp::kSchemaHash;
    frc::SmartDashboard::PutString("Network/SchemaHash", hashStr.str());
    
    // Update current command info
    auto cmd = GetCommand();
    if (cmd) {
        frc::SmartDashboard::PutNumber("Network/CmdVx", cmd->vx.value());
        frc::SmartDashboard::PutNumber("Network/CmdW", cmd->omega.value());
    } else {
        frc::SmartDashboard::PutNumber("Network/CmdVx", 0.0);
        frc::SmartDashboard::PutNumber("Network/CmdW", 0.0);
    }

    // Flush telemetry to host if connected and interval elapsed
    if (IsConnected()) {
        m_telemetryAccumulator.MaybeFlush(*m_tcpAdapter);
    }
    
    // Publish telemetry stats
    auto telemMetrics = m_telemetryAccumulator.GetMetrics();
    frc::SmartDashboard::PutNumber("Network/TelemetrySent", static_cast<double>(telemMetrics.messagesSent));
    frc::SmartDashboard::PutNumber("Network/TelemetryPackets", static_cast<double>(telemMetrics.packetsSent));
}

std::optional<Netman::Command> Netman::GetCommand() {
    // Get active command from queue
    auto bcnpCmd = m_driveQueue.ActiveMessage();
    
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
    return m_driveQueue.IsConnected(now) && m_tcpAdapter->IsConnected();
}

void Netman::ClearQueue() {
    m_driveQueue.Clear();
}

void Netman::RecordTelemetry(float leftVel, float rightVel, float leftPos, float rightPos) {
    bcnp::DrivetrainTelemetry telem{
        .leftVelocity = leftVel,
        .rightVelocity = rightVel,
        .leftPosition = leftPos,
        .rightPosition = rightPos,
        .timestampMs = static_cast<uint32_t>(frc::Timer::GetFPGATimestamp().value() * 1000.0)
    };
    m_telemetryAccumulator.Record(telem);
}