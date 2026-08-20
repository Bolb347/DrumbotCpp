#pragma once
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
public:
    Robot();

    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

    frc::SendableChooser<std::string> sideChooser;

private:
    void RobotInit();

    frc2::Command* m_autonomousCommand = nullptr;
    RobotContainer m_robotContainer;
};
