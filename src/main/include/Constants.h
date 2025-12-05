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
    constexpr double kP = 0.1;        // prop gain
    constexpr double kI = 0.0;           // integ gain
    constexpr double kD = 0.0;           // d/dx gain
    constexpr double kFF = 1.0 / kMaxSpeed.value();     // feedforward (1/max_rpm)
    
    // enc conv factors
    // for brushed motors with built-in encs
    // adjust based on gearing and wheel diameter (todo)
    constexpr double kWheelDiameterMeters = 0.1524;  // 6 inches
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

