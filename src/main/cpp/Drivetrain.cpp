#include "Drivetrain.h"
#include <rev/config/SparkMaxConfig.h>
#include <algorithm>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

using rev::spark::SparkBaseConfig;
using rev::spark::SparkBase;
using rev::spark::SparkMaxConfig;
using rev::spark::SparkMax;

Drivetrain::Drivetrain()
    : m_leftLeader(DriveConstants::kLeftLeaderCanId, SparkMax::MotorType::kBrushed),
      m_leftFollower(DriveConstants::kLeftFollowerCanId, SparkMax::MotorType::kBrushed),
      m_rightLeader(DriveConstants::kRightLeaderCanId, SparkMax::MotorType::kBrushed),
      m_rightFollower(DriveConstants::kRightFollowerCanId, SparkMax::MotorType::kBrushed),
      m_leftEncoder(m_leftLeader.GetEncoder()),
      m_rightEncoder(m_rightLeader.GetEncoder()),
      m_leftPID(m_leftLeader.GetClosedLoopController()),
      m_rightPID(m_rightLeader.GetClosedLoopController())
{    
    SparkMaxConfig baseConfig;
    baseConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    baseConfig.SmartCurrentLimit(40);
    
    // conf enc conv factors
    baseConfig.encoder
        .PositionConversionFactor(DriveConstants::kPositionConversionFactor)
        .VelocityConversionFactor(DriveConstants::kVelocityConversionFactor);
    
    // conf PID for vel cont (slot 0)
    baseConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(DriveConstants::kP, DriveConstants::kI, DriveConstants::kD)
        .VelocityFF(DriveConstants::kFF)
        .OutputRange(-1.0, 1.0);

    constexpr auto kReset   = SparkBase::ResetMode::kNoResetSafeParameters;
    constexpr auto kPersist = SparkBase::PersistMode::kPersistParameters;

    // conf left side
    m_leftLeader.Configure(baseConfig, kReset, kPersist);

    {
        SparkMaxConfig cfg;
        cfg.Apply(baseConfig);
        cfg.Follow(m_leftLeader);
        m_leftFollower.Configure(cfg, kReset, kPersist);
    }

    // conf right side (inverted)
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

void Drivetrain::ArcadeDrive(double fwd, double rot, bool enableSlewRate) {
    // switch to open-loop percent out mode for teleop
    m_usingVelocityControl = false;
    
    double fwdLimited = fwd;
    double rotLimited = rot;
    
    // only apply slew rate limit in teleop
    if (enableSlewRate) {
        fwdLimited = m_driveLimiter.Calculate(units::dimensionless::scalar_t{fwd}).value();
        rotLimited = m_turnLimiter.Calculate(units::dimensionless::scalar_t{rot}).value();
    } else {
        m_driveLimiter.Reset(fwd);
        m_turnLimiter.Reset(rot);
    }

    double left = std::clamp(fwdLimited + rotLimited, -1.0, 1.0);
    double right = std::clamp(fwdLimited - rotLimited, -1.0, 1.0);

    // use percent out (open-loop)
    m_leftLeader.Set(left);
    m_rightLeader.Set(right);

    frc::SmartDashboard::PutNumber("Drivetrain/LeftOutput", left);
    frc::SmartDashboard::PutNumber("Drivetrain/RightOutput", right);
}

void Drivetrain::DriveFromVelocity(units::meters_per_second_t vx, units::radians_per_second_t omega) {    
    m_usingVelocityControl = true;
    
    double fwd_mps = vx.value();
    double rot_rps = omega.value();
    
    // clamp to phys lims
    fwd_mps = std::clamp(fwd_mps, -DriveConstants::kMaxSpeed.value(), 
                                    DriveConstants::kMaxSpeed.value());
    rot_rps = std::clamp(rot_rps, -DriveConstants::kMaxAngularSpeed.value(), 
                                    DriveConstants::kMaxAngularSpeed.value());
    
    // approx arcade drive mixing
    double fwd_norm = fwd_mps / DriveConstants::kMaxSpeed.value();
    double rot_norm = rot_rps / DriveConstants::kMaxAngularSpeed.value();
    
    double left_norm = std::clamp(fwd_norm + rot_norm, -1.0, 1.0);
    double right_norm = std::clamp(fwd_norm - rot_norm, -1.0, 1.0);
    
    // conv to real vels in m/s
    double left_velocity_mps = left_norm * DriveConstants::kMaxSpeed.value();
    double right_velocity_mps = right_norm * DriveConstants::kMaxSpeed.value();
    
    // use SM closed-loop vel cont
    // SetReference w/kVel uses PID contrlr conf'd
    m_leftPID.SetReference(left_velocity_mps, rev::spark::SparkMax::ControlType::kVelocity);
    m_rightPID.SetReference(right_velocity_mps, rev::spark::SparkMax::ControlType::kVelocity);
    
    // pub telem
    frc::SmartDashboard::PutNumber("Drivetrain/LeftVelCmd", left_velocity_mps);
    frc::SmartDashboard::PutNumber("Drivetrain/RightVelCmd", right_velocity_mps);
}

void Drivetrain::Stop() {
    if (m_usingVelocityControl) {
        // stop using vel cont
        m_leftPID.SetReference(0.0, rev::spark::SparkMax::ControlType::kVelocity);
        m_rightPID.SetReference(0.0, rev::spark::SparkMax::ControlType::kVelocity);
    } else {
        // stop using per out
        m_leftLeader.Set(0.0);
        m_rightLeader.Set(0.0);
    }
}

void Drivetrain::Periodic() {
    // pub enc feedbk
    frc::SmartDashboard::PutNumber("Drivetrain/LeftVelActual", m_leftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Drivetrain/RightVelActual", m_rightEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Drivetrain/LeftPos", m_leftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Drivetrain/RightPos", m_rightEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("Drivetrain/VelocityMode", m_usingVelocityControl);
}