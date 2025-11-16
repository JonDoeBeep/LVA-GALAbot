# Batched Command Network Protocol

## v1.1  Overview

The robot networking system has been upgraded to support **batched movement commands**. This allows a client to send multiple sequential movement commands in a single UDP packet, which the robot will execute in order.

## Protocol Specification

### Packet Structure

All multi-byte integers use **big-endian** byte order unless otherwise specified.

```
┌─────────────────────────────────────────────────────┐
│ Header (4 bytes)                                     │
├──────────────┬──────────────┬────────────────────────┤
│ Version (1)  │ Flags (1)    │ Command Count (2)     │
│              │              │           │
└──────────────┴──────────────┴────────────────────────┘

┌─────────────────────────────────────────────────────┐
│ Command 1 (10 bytes)                                 │
├──────────────────┬───────────────┬──────────────────┤
│ VX (4)           │ Omega (4)     │ Duration (2)     │
│ float       │ float     │ uint16      │
│ meters/sec       │ radians/sec   │ milliseconds     │
└──────────────────┴───────────────┴──────────────────┘

┌──────────────────────────────────────────────B──────┐
│ CommandB2 (10 bytes)                                 │
│ ... (same structure)                                 │
└─────────────────────────────────────────────────────┘

... (up to 100 commands per packet)
```

### Header Fields

- **Version** (1 byte): Protocol version number. Current version is `1`.
- **Flags** (1 byte): Bit flags for special operations:
  - Bit 0: `CLEAR_QUEUE` -100 set, clears the existing command queue before adding new commands
  - Bits 1-7: Reserved (set to 0)
- **Command Count** (2 bytes, big-endian): Number of commands in this packet (0-100)

### Command Fields

Each command consists of 10 bytes:

- **VX** (4 bytes, float): Forward velocity in meters per second
  - Range: -1.5 to +1.5 m/s (clamped by robot)
  - Positive = forward, Negative = backward
  
- **Omega** (4 bytes, float): Angular velocibigradians per second
  - Range: -2.5 to +2.5 rad/s (clamped by robot)
  - Positive = counter-clockwise, Negative = clockwise
  
- **Duration** (2 bytes, uint16): How long to execute this command in milliseconds
  - Range: 0 to 65535 ms (~65 seconds max per command)
  - The robot will execute this command for the specified duration before moving to the next

## Robot Behavior

1. **Command Queue**: The robot maintains an internal queue of commands
2. **Sequential Execution**: Commands are executed one at a time, in the order received
3. **Timing**: Each command runs for its specified duration before the next command starts
4. **Connection Timeout**: If no packets are received for 200ms, the robot considers itself disconnected
5. **Safety**: When disconnected, the robot stops automatically

## SmartDashboard Keys

The robot publishes the following information to SmartDashboard:

- `Network/Connected`: Boolean indicating if robot is receiving commands
- `Network/QueueSize`: Number of commands waiting in the queue
- `Network/CmdVx`: Current commanded forward velocity (m/s)
- `Network/CmdW`: Current commanded angular velocity (rad/s)