#pragma once

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/length.h>
#include <units/base.h>
#include <numbers>


namespace DriveConstants {
    // pdh can id 1
    constexpr int kLeftLeaderCanId = 2;
    constexpr int kLeftFollowerCanId = 3;
    constexpr int kRightLeaderCanId = 4;
    constexpr int kRightFollowerCanId = 5;

    // phys bot params
    constexpr auto kMaxSpeed = 1.5_mps;
    constexpr auto kMaxAngularSpeed = 2.5_rad_per_s;
    
    // cl vel PID consts
    // need to be tuned for robot!
    constexpr double kP = 0.0001;        // prop gain
    constexpr double kI = 0.0;           // integ gain
    constexpr double kD = 0.0;           // d/dx gain
    constexpr double kFF = 0.000156;     // feedforward (1/max_rpm)
    
    // enc conv factors
    // for brushed motors with built-in encs
    // adjust based on gearing and wheel diameter (todo)
    constexpr double kWheelDiameterMeters = 0.1524_m;  // 6 inches
    constexpr double kGearRatio = 10.71;              // adj for gearbox
    constexpr double kEncoderCPR = 42.0;             // counts per revolution
    
    // calc conv factor: motor rots -> meters
    constexpr double kPositionConversionFactor = 
        (kWheelDiameterMeters * std::numbers::pi) / kGearRatio;
    
    // calc conv fac: motor RPM -> meters/second  
    constexpr double kVelocityConversionFactor = kPositionConversionFactor / 60.0;
    
    // slew rates
    constexpr auto kDriveSlewRate = 2.0 / 1_s;
    constexpr auto kTurnSlewRate = 3.0 / 1_s;
}

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
}

namespace NetworkConstants {
    // udp port - WARNING: UDP is ass. will be replaced with spi
    // packet loss and out-of-order delivery can occur
    constexpr int kUdpPort = 5808;
    
    // con timeout
    constexpr auto kCommandTimeout = 0.2_s;
    
    // protocol conts - BCNP v1.1.0 (semantic versioning)
    // Breaking changes: all fields now big-endian
    constexpr uint8_t kProtocolMajor = 1;
    constexpr uint8_t kProtocolMinor = 1;
    constexpr uint8_t kProtocolPatch = 0;
    constexpr size_t kMaxPacketSize = 1024;
    constexpr size_t kHeaderSize = 4;
    constexpr size_t kCommandSize = 10;
    constexpr size_t kMaxCommandsPerPacket = 100;
    constexpr size_t kMaxQueueSize = 200; // prevent DOS
    
    // flag bits
    constexpr uint8_t kFlagClearQueue = 0x01;
}