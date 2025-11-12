# BCNP: Batched Command Network Protocol

## Overview

The robot networking system has been upgraded to support new batched movement commands. This allows a client to send multiple sequential movement commands in a single UDP packet, which the robot will execute in order. This helps because it allows us to send full autonomous routines to the robot, make nonlinear trajectories smoother by breaking it into parts, and makes us not give a shit about jitter and other network shit.

## Protocol Specification

### Packet Structure

All multi-byte integers use big-endian byte order unless otherwise specified.

```
┌─────────────────────────────────────────────────────┐
│ Header (4 bytes)                                     │
├──────────────┬──────────────┬────────────────────────┤
│ Version (1)  │ Flags (1)    │ Command Count (2)     │
│              │              │ (big-endian)           │
└──────────────┴──────────────┴────────────────────────┘

┌─────────────────────────────────────────────────────┐
│ Command 1 (10 bytes)                                 │
├──────────────────┬───────────────┬──────────────────┤
│ VX (4)           │ Omega (4)     │ Duration (2)     │
│ float (LE)       │ float (LE)    │ uint16 (BE)      │
│ meters/sec       │ radians/sec   │ milliseconds     │
└──────────────────┴───────────────┴──────────────────┘

┌─────────────────────────────────────────────────────┐
│ Command 2 (10 bytes)                                 │
│ ... (same structure)                                 │
└─────────────────────────────────────────────────────┘

... (up to 100 commands per packet)
```

### Header Fields

- **Version** (1 byte): Protocol version number. Current version is `1`.
- **Flags** (1 byte): Bit flags for special operations:
  - Bit 0: `CLEAR_QUEUE` - If set, clears the existing command queue before adding new commands
  - Bits 1-7: Reserved (set to 0)
- **Command Count** (2 bytes, big-endian): Number of commands in this packet (0-100)

### Command Fields

Each command consists of 10 bytes:

- **VX** (4 bytes, float, little-endian): Forward velocity in meters per second
  - Range: -1.5 to +1.5 m/s (clamped by robot)
  - Positive = forward, Negative = backward
  
- **Omega** (4 bytes, float, little-endian): Angular velocity in radians per second
  - Range: -2.5 to +2.5 rad/s (clamped by robot)
  - Positive = counter-clockwise, Negative = clockwise
  
- **Duration** (2 bytes, uint16, big-endian): How long to execute this command in milliseconds
  - Range: 0 to 65535 ms (~65 seconds max per command)
  - The robot will execute this command for the specified duration before moving to the next

## SmartDashboard Keys

The robot publishes the following info to SmartDashboard:

- `Network/Connected`: Boolean indicating if robot is receiving commands
- `Network/QueueSize`: Number of commands waiting in the queue
- `Network/CmdVx`: Current commanded forward velocity (m/s)
- `Network/CmdW`: Current commanded angular velocity (rad/s)