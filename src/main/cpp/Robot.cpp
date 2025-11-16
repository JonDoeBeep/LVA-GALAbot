#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

void Robot::RobotPeriodic() {
    // check for new pkts no matter what
    m_Netman.Periodic();
    
    frc::SmartDashboard::PutBoolean("Network/Connected", m_Netman.IsConnected());
    frc::SmartDashboard::PutNumber("Network/QueueSize", m_Netman.GetQueueSize());
}

void Robot::AutonomousInit() {
    // when startstop in the name of law
    m_drivetrain.Stop();
}

void Robot::AutonomousPeriodic() {
    if (auto cmd = m_Netman.GetCommand()) {
        m_drivetrain.DriveFromVelocity(cmd->vx, cmd->omega);
        frc::SmartDashboard::PutNumber("Network/CmdVx", cmd->vx.value());
        frc::SmartDashboard::PutNumber("Network/CmdW",  cmd->omega.value());
    } else {
        // when connection lost stop in the name of law
        m_drivetrain.Stop();
    }
}

void Robot::TeleopInit() {
    m_drivetrain.Stop(); // in the name of the law
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