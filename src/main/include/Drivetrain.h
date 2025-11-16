#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"

class Drivetrain {
public:
    Drivetrain();
    
    // teleop cont with norm'd [-1,1] contrlr vals
    void ArcadeDrive(double fwd, double rot, bool enableSlewRate = true);
    
    // closed-loop vel con using sm PID
    // uses enc fdbk to keep com vel
    // regardless of bat v, fric, or def
    void DriveFromVelocity(units::meters_per_second_t vx, units::radians_per_second_t omega);

    void Stop();

    void Periodic();

private:
    rev::spark::SparkMax m_leftLeader;
    rev::spark::SparkMax m_leftFollower;
    rev::spark::SparkMax m_rightLeader;
    rev::spark::SparkMax m_rightFollower;
    
    rev::spark::SparkRelativeEncoder m_leftEncoder;
    rev::spark::SparkRelativeEncoder m_rightEncoder;
    
    rev::spark::SparkClosedLoopController m_leftPID;
    rev::spark::SparkClosedLoopController m_rightPID;

    frc::SlewRateLimiter<units::dimensionless::scalar> m_driveLimiter{DriveConstants::kDriveSlewRate};
    frc::SlewRateLimiter<units::dimensionless::scalar> m_turnLimiter{DriveConstants::kTurnSlewRate};
    
    bool m_usingVelocityControl{false};
};