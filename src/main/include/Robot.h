#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include "Constants.h"
#include "Drivetrain.h"
#include "Netman.h"

class Robot : public frc::TimedRobot {
public:
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;

private:
    Drivetrain m_drivetrain;
    Netman m_Netman;
    frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
};