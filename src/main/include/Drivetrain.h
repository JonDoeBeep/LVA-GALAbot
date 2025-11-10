#pragma once

#include <rev/SparkMax.h>
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"

class Drivetrain {
public:
    Drivetrain();
    
    // main meth for teleop takes normalized [-1,1] joystick values.
    void ArcadeDrive(double fwd, double rot);
    
    // main method for auth takes phys units like m/s or rad/s.
    void DriveFromVelocity(units::meters_per_second_t vx, units::radians_per_second_t omega);

    // stop in the name of law
    void Stop();

    void Periodic();

private:
    void ConfigureMotor(rev::spark::SparkMax& motor, bool inverted);

    rev::spark::SparkMax m_leftLeader;
    rev::spark::SparkMax m_leftFollower;
    rev::spark::SparkMax m_rightLeader;
    rev::spark::SparkMax m_rightFollower;

    frc::SlewRateLimiter<units::dimensionless::scalar> m_driveLimiter{DriveConstants::kDriveSlewRate};
    frc::SlewRateLimiter<units::dimensionless::scalar> m_turnLimiter{DriveConstants::kTurnSlewRate};
};