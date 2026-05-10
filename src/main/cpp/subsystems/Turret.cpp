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
    target /= 360.0;

    if (!RobotContainer::isBlueAlliance.GetValue()) {
        target += 0.5;
    }

    double robotRotation = m_drivetrain->GetState().Pose.Rotation().Radians().value() / (2.0 * std::numbers::pi);
    double robotRelativeTarget = target - robotRotation;
    robotRelativeTarget = robotRelativeTarget - std::floor(robotRelativeTarget + 0.5);

    constexpr double kLimit = 0.75;

    double candidates[3] = {
        robotRelativeTarget - 1.0,
        robotRelativeTarget,
        robotRelativeTarget + 1.0
    };

    double best = std::numeric_limits<double>::max();
    double bestDist = std::numeric_limits<double>::max();

    for (double c : candidates) {
        if (c >= -kLimit && c <= kLimit) {
            double dist = std::abs(c - m_target);
            if (dist < bestDist) {
                bestDist = dist;
                best = c;
            }
        }
    }

    if (best == std::numeric_limits<double>::max()) {
        best = std::clamp(candidates[1], -kLimit, kLimit);
    }

    goToTarget(best);
}

void Turret::goToTarget(double target) {
    m_target = target;
}

// This method will be called once per scheduler run
void Turret::Periodic() {
    m_spinner->SetControl(controls::PositionVoltage{units::turn_t{m_target * m_ratio}});
}

double Turret::getAngleDegrees() {
    return m_spinner->GetRotorPosition().GetValueAsDouble() * 360;
}