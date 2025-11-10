#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

void Robot::RobotPeriodic() {
    // check for new pkts no matter what
    m_Netman.Periodic();
    
    frc::SmartDashboard::PutBoolean("Network/Connected", m_Netman.IsConnected());
}

void Robot::AutonomousInit() {
    // when startstop in the name of law
    m_drivetrain.Stop();
}

void Robot::AutonomousPeriodic() {
    if (m_Netman.IsConnected()) {
        auto cmd = m_Netman.GetCommand();
        if (cmd.has_value()) {
            m_drivetrain.DriveFromVelocity(cmd->vx, cmd->omega);
            frc::SmartDashboard::PutNumber("Network/CmdVx", cmd->vx.value());
            frc::SmartDashboard::PutNumber("Network/CmdW", cmd->omega.value());
        }
    } else {
        // when connection lost stop in the name of law
        m_drivetrain.Stop();
    }
}

void Robot::TeleopInit() {
    m_drivetrain.Stop();
}

void Robot::TeleopPeriodic() {
    // simple db
    auto db = [](double x, double dz){ return (std::abs(x) < dz) ? 0.0 : x; };

    // db joystick
    double forward = -db(m_driverController.GetLeftY(), 0.08);
    double rotation = db(m_driverController.GetRightX(), 0.08);

    m_drivetrain.ArcadeDrive(forward, rotation);
}

void Robot::DisabledInit() {
    // stop in the name of law
    m_drivetrain.Stop();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif