#pragma once

#include "Constants.h"
#include <bcnp/message_types.h>
#include <bcnp/message_queue.h>
#include <bcnp/dispatcher.h>
#include <bcnp/transport/tcp_posix.h>
#include <bcnp/transport/controller_driver.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include <memory>
#include <optional>

/**
 * @brief Network manager for receiving drive commands over BCNP protocol.
 * 
 * Handles TCP connections, parses BCNP packets, and provides drive commands
 * to the drivetrain subsystem through a timed message queue.
 */
class Netman {
public:
    /// TCP port for BCNP communication
    static constexpr uint16_t kTcpPort = 5800;

    /// Processed drive command with WPILib units
    struct Command {
        units::meters_per_second_t vx{0};
        units::radians_per_second_t omega{0};
        units::millisecond_t duration{0};
    };

    Netman();
    ~Netman();

    /// Call periodically to process incoming packets and update queue
    void Periodic();

    /// Get the currently active drive command
    std::optional<Command> GetCommand();

    /// Check if network connection is established
    bool IsConnected();

    /// Get the number of commands in queue
    std::size_t GetQueueSize() const { return m_driveQueue.Size(); }

    /// Clear all queued commands
    void ClearQueue();

private:
    bcnp::MessageQueue<bcnp::DriveCmd> m_driveQueue;
    bcnp::PacketDispatcher m_dispatcher;
    std::unique_ptr<bcnp::TcpPosixAdapter> m_tcpAdapter;
    std::unique_ptr<bcnp::DispatcherDriver> m_driver;
};