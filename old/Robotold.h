// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

// Use REV CANSparkMax if available; otherwise fall back to PWMSparkMax so
// the project can still compile when the vendor library isn't installed.
#if defined(__has_include)
#if __has_include(<rev/CANSparkMax.h>)
#include <rev/CANSparkMax.h>
#define HAS_REV_CANSparkMax 1
#else
#include <frc/motorcontrol/PWMSparkMax.h>
#define HAS_REV_CANSparkMax 0
#endif
#else
// If __has_include is not supported, assume REV is not available.
#include <frc/motorcontrol/PWMSparkMax.h>
#define HAS_REV_CANSparkMax 0
#endif

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // Motor controllers - prefer CAN (REV CANSparkMax) when available.
  // If REV headers are missing, this falls back to PWMSparkMax so the
  // project still builds. Adjust CAN IDs or PWM ports as needed.
#if HAS_REV_CANSparkMax
  std::vector<rev::CANSparkMax*> m_motors;
  rev::CANSparkMax m_leftMotor{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightMotor{2, rev::CANSparkMax::MotorType::kBrushless};
#else
  std::vector<frc::PWMSparkMax*> m_motors;
  frc::PWMSparkMax m_leftMotor{0};
  frc::PWMSparkMax m_rightMotor{1};
#endif
};
