#include "robot/subsystems/CommandSwerveDrivetrain.h"

#include "ctre/phoenix6/Utils.hpp"
#include "robot/RobotContainer.h"
#include "robot/Constants.h"
#include "robot/generated/TunerConstants.h"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <units/math.h>
#include <units/time.h>
#include <numbers>
#include <cmath>
#include <frc/RobotController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

using namespace subsystems;

void CommandSwerveDrivetrain::ConfigureAutoBuilder()
{
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        // Supplier of current robot pose
        [this] { return GetState().Pose; },
        // Consumer for seeding pose against auto
        [this](frc::Pose2d const &pose) { return ResetPose(pose); },
        // Supplier of current robot speeds
        [this] { return GetState().Speeds; },
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        [this](frc::ChassisSpeeds const &speeds, pathplanner::DriveFeedforwards const &feedforwards) {
            return SetControl(
                m_pathApplyRobotSpeeds.WithSpeeds(frc::ChassisSpeeds::Discretize(speeds, 20_ms))
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
            );
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // PID constants for translation
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            // PID constants for rotation
            pathplanner::PIDConstants{7.0, 0.0, 0.0}
        ),
        std::move(config),
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        [] {
            auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
            return alliance == frc::DriverStation::Alliance::kRed;
        },
        this // Subsystem for requirements
    );
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

frc2::CommandPtr CommandSwerveDrivetrain::PointInOrientation(
    frc2::CommandPS5Controller* controller, double targetDegrees)
{
    return ApplyRequest([this, controller, targetDegrees]() -> auto&& {
        static swerve::requests::FieldCentricFacingAngle req{};
        req.WithTargetDirection(frc::Rotation2d{units::degree_t{targetDegrees}})
           .WithVelocityX(units::meters_per_second_t{-controller->GetLeftY() * TunerConstants::kSpeedAt12Volts.value() * 0.5})
           .WithVelocityY(units::meters_per_second_t{-controller->GetLeftX() * TunerConstants::kSpeedAt12Volts.value() * 0.5})
           .WithHeadingPID(0.1, 0, 0)
           .WithRotationalDeadband(units::radians_per_second_t{0.2})
           .WithDriveRequestType(swerve::DriveRequestType::Velocity);
        return req;
    });
}

void CommandSwerveDrivetrain::Periodic() {
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance.has_value()) {
            SetOperatorPerspectiveForward(
                alliance.value() == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation);
            m_hasAppliedOperatorPerspective = true;
        }
    }

    if (frc::DriverStation::IsTeleopEnabled()) {
        ShootingState state = GetShootingState();
        auto alliance = frc::DriverStation::GetAlliance();
        bool isBlue = !alliance.has_value() ||
                      alliance.value() == frc::DriverStation::Alliance::kBlue;

        if (state == ShootingState::PASSINGLEFT) {
            RobotContainer::isPassing = true;
            RobotContainer::target    = isBlue ? RobotContainer::passTarget1Blue
                                               : RobotContainer::passTarget1Red;
        } else if (state == ShootingState::PASSINGRIGHT) {
            RobotContainer::isPassing = true;
            RobotContainer::target    = isBlue ? RobotContainer::passTarget2Blue
                                               : RobotContainer::passTarget2Red;
        } else {
            RobotContainer::isPassing = false;
            RobotContainer::target    = isBlue ? RobotContainer::scoreTargetBlue
                                               : RobotContainer::scoreTargetRed;
        }
    }
}

int CommandSwerveDrivetrain::GetSafeShootZone() const {
    double poseX = GetState().Pose.X().value();
    double poseY = GetState().Pose.Y().value();
    auto alliance = frc::DriverStation::GetAlliance();
    bool isBlue = !alliance.has_value() ||
                  alliance.value() == frc::DriverStation::Alliance::kBlue;

    if (isBlue) {
        return poseX > 2.313
            ? (poseY > 5.391 ? 2 : (poseY > 2.696 ? 0 : 1))
            : (poseY > 5.391 ? 3 : (poseY > 2.696 ? 5 : 4));
    } else {
        return poseX < 14.138
            ? (poseY > 5.391 ? 1 : (poseY > 2.696 ? 0 : 2))
            : (poseY > 5.391 ? 4 : (poseY > 2.696 ? 5 : 3));
    }
}

bool CommandSwerveDrivetrain::IsInHoodDeadzone() const {
    double poseX = GetState().Pose.X().value();
    double poseY = GetState().Pose.Y().value();
    frc::ChassisSpeeds vel = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        GetState().Speeds, GetState().Pose.Rotation());

    double vx = vel.vx.value();
    if ((poseX > (4.5 - std::abs(vx * 0.2)) && poseX < (4.7 + std::abs(vx * 0.2))) ||
        (poseX > (11.8 - std::abs(vx * 0.2)) && poseX < (12.0 + std::abs(vx * 0.2)))) {
        if ((poseY > 0 && poseY < 1.4) || (poseY > 6.4 && poseY < 8.2)) {
            return true;
        }
    }
    return false;
}

CommandSwerveDrivetrain::ShootingState CommandSwerveDrivetrain::GetShootingState() const {
    double poseX = GetState().Pose.X().value();
    double poseY = GetState().Pose.Y().value();
    auto alliance = frc::DriverStation::GetAlliance();
    bool isBlue = !alliance.has_value() ||
                  alliance.value() == frc::DriverStation::Alliance::kBlue;

    bool cond = isBlue ? (poseX > 4.7) : (poseX < 12.0);
    if (cond) {
        return poseY < 4.0 ? ShootingState::PASSINGRIGHT : ShootingState::PASSINGLEFT;
    }
    return ShootingState::SCORING;
}

frc::Pose3d CommandSwerveDrivetrain::GetPositionRelativeField() const {
    frc::Pose2d   robotPose     = GetState().Pose;
    frc::Rotation2d robotRotation = robotPose.Rotation();
    frc::Translation2d turretOffset{0.1_m, 0_m};
    frc::Translation2d turretLoc2d = robotPose.Translation() +
                                     turretOffset.RotateBy(robotRotation);
    frc::Translation3d turretPos{turretLoc2d.X(), turretLoc2d.Y(), 0.1_m};
    return frc::Pose3d{turretPos, frc::Rotation3d{0_rad, 0_rad, robotRotation.Radians()}};
}
