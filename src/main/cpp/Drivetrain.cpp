#include "Drivetrain.h"
#include <rev/config/SparkMaxConfig.h>
#include <algorithm>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

using rev::spark::SparkBaseConfig;
using rev::spark::SparkBase;
using rev::spark::SparkMaxConfig;

Drivetrain::Drivetrain()
    : m_leftLeader(DriveConstants::kLeftLeaderCanId, rev::spark::SparkMax::MotorType::kBrushed),
      m_leftFollower(DriveConstants::kLeftFollowerCanId, rev::spark::SparkMax::MotorType::kBrushed),
      m_rightLeader(DriveConstants::kRightLeaderCanId, rev::spark::SparkMax::MotorType::kBrushed),
      m_rightFollower(DriveConstants::kRightFollowerCanId, rev::spark::SparkMax::MotorType::kBrushed)
{    
    SparkMaxConfig baseConfig;
    baseConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    baseConfig.SmartCurrentLimit(40);


    constexpr auto kReset   = SparkBase::ResetMode::kNoResetSafeParameters;
    constexpr auto kPersist = SparkBase::PersistMode::kPersistParameters;

    m_leftLeader.Configure(baseConfig, kReset, kPersist);

    {
        SparkMaxConfig cfg;
        cfg.Apply(baseConfig);
        cfg.Follow(m_leftLeader);
        m_leftFollower.Configure(cfg, kReset, kPersist);
    }

    {
        SparkMaxConfig cfg;
        cfg.Apply(baseConfig);
        cfg.Inverted(true);
        m_rightLeader.Configure(cfg, kReset, kPersist);
    }

    {
        SparkMaxConfig cfg;
        cfg.Apply(baseConfig);
        cfg.Follow(m_rightLeader);
        m_rightFollower.Configure(cfg, kReset, kPersist);
    }
}

void Drivetrain::ArcadeDrive(double fwd, double rot) {
    auto fwdLimited = m_driveLimiter.Calculate(units::dimensionless::scalar_t{fwd}).value();
    auto rotLimited = m_turnLimiter.Calculate(units::dimensionless::scalar_t{rot}).value();

    double left = std::clamp(fwdLimited + rotLimited, -1.0, 1.0);
    double right = std::clamp(fwdLimited - rotLimited, -1.0, 1.0);

    m_leftLeader.Set(left);
    m_rightLeader.Set(right);

    frc::SmartDashboard::PutNumber("Drivetrain/LeftOutput", left);
    frc::SmartDashboard::PutNumber("Drivetrain/RightOutput", right);
}

void Drivetrain::DriveFromVelocity(units::meters_per_second_t vx, units::radians_per_second_t omega) {
    double fwd_norm = (vx / DriveConstants::kMaxSpeed).value();
    double rot_norm = (omega / DriveConstants::kMaxAngularSpeed).value();
    
    ArcadeDrive(fwd_norm, rot_norm);
}

void Drivetrain::Stop() {
    m_leftLeader.Set(0.0);
    m_rightLeader.Set(0.0);
}

void Drivetrain::Periodic() {
    // todo add odomoetry and shit i guess
}