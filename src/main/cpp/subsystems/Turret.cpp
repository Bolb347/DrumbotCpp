// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "robot/subsystems/Turret.h"
#include <robot/Constants.h>
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include "robot/RobotContainer.h"

using namespace subsystems;
using namespace ctre::phoenix6;

Turret::Turret(hardware::TalonFX* spinner, CommandSwerveDrivetrain *drivetrain) {
    m_drivetrain = drivetrain;
    m_spinner = spinner;
    m_spinner->GetConfigurator().Apply(constants::TurretConstants::BuildSpinnerConfig());
}

void Turret::goToTargetFieldRelative(double target) {
    double robotRotation = m_drivetrain->GetState().Pose.Rotation().Radians().value() / (2.0 * std::numbers::pi);

    if (!RobotContainer::isBlueAlliance.GetValue()) {
        target += 0.5;
    }
    double robotRelativeTarget = target - robotRotation;
    goToTarget(robotRelativeTarget);
}

void Turret::goToTarget(double target) {
    m_target = target;
}

// This method will be called once per scheduler run
void Turret::Periodic() {
    m_spinner->SetControl(controls::PositionVoltage{units::turn_t{m_target * m_ratio}});
}
