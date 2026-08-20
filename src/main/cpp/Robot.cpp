// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "robot/Robot.h"

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableInstance.h>
#include <pathplanner/lib/commands/FollowPathCommand.h>

Robot::Robot() {
    // RobotContainer constructor performs all subsystem/button setup
    // (m_robotContainer is constructed by the member initializer)
    RobotInit();

    sideChooser.SetDefaultOption("No Override", "No Override");
    sideChooser.AddOption("Blue", "Blue");
    sideChooser.AddOption("Red",  "Red");
    frc::SmartDashboard::PutData("Side Chooser", &sideChooser);
}

void Robot::RobotInit() {
    frc::SmartDashboard::PutBoolean("IsBlue", RobotContainer::isBlueAlliance.GetValue());
    ctre::phoenix6::SignalLogger::EnableAutoLogging(false);
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
    frc::SmartDashboard::PutNumber("Batt", frc::RobotController::GetBatteryVoltage().value());
    frc::SmartDashboard::PutBoolean("IsBlue", RobotContainer::isBlueAlliance.GetValue());

    std::string selected = sideChooser.GetSelected();
    if (frc::DriverStation::IsDisabled()) {
        if (selected.empty() || selected == "No Override") {
            auto alliance = frc::DriverStation::GetAlliance();
            bool isBlue = alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kBlue;
            RobotContainer::isBlueAlliance.SetValue(isBlue);
        } else {
            RobotContainer::isBlueAlliance.SetValue(selected == "Blue");
        }
    }
}

void Robot::DisabledInit() {
    nt::NetworkTableInstance::GetDefault()
        .GetTable("LiveWindow")
        ->GetEntry("MatchActive")
        .SetBoolean(false);
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
    nt::NetworkTableInstance::GetDefault()
        .GetTable("LiveWindow")
        ->GetEntry("MatchActive")
        .SetBoolean(true);

    frc2::CommandScheduler::GetInstance().CancelAll();
    frc::SmartDashboard::PutBoolean("IsBlue", RobotContainer::isBlueAlliance.GetValue());

    m_autonomousCommand = m_robotContainer.GetAutonomousCommand();
    if (m_autonomousCommand != nullptr) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    frc::SmartDashboard::PutBoolean("IsBlue", RobotContainer::isBlueAlliance.GetValue());
    if (m_autonomousCommand != nullptr) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
