#pragma once

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/base.h>

namespace DriveConstants {
    // pdh can id 1
    constexpr int kLeftLeaderCanId = 2;
    constexpr int kLeftFollowerCanId = 3;
    constexpr int kRightLeaderCanId = 4;
    constexpr int kRightFollowerCanId = 5;

    // dont nuke self
    constexpr auto kMaxSpeed = 1.5_mps;
    constexpr auto kMaxAngularSpeed = 2.5_rad_per_s;
    
    // slew rates (how quickly the robot can accelerate)
    // percent output per sec how fast it goes so like 2.0/1s means 0-100 in .5s
    constexpr auto kDriveSlewRate = 2.0 / 1_s;
    constexpr auto kTurnSlewRate = 3.0 / 1_s;
}

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
}

namespace NetworkConstants {
    // udp port
    constexpr int kUdpPort = 5808;
    // con timeout
    constexpr auto kCommandTimeout = 0.2_s;
}