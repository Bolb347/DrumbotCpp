#pragma once

#include <optional>
#include <vector>
#include <string>
#include <functional>

#include "ctre/phoenix6/swerve/SwerveRequest.hpp"

#include <frc/GenericHID.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>
#include <frc2/command/button/Trigger.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/json_fwd.h>

#include "robot/Telemetry.h"
#include "robot/util/ObservableField.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include "robot/subsystems/Hopper.h"
#include "robot/subsystems/Intake.h"
#include "robot/subsystems/SuperStructure.h"
#include "robot/subsystems/Vision.h"
#include "robot/generated/TunerConstants.h"

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <vector>
#include <memory>

using namespace ctre::phoenix6;

class RobotContainer {
public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();

    // ── Shared robot state ──
    static double MaxSpeed;
    static double MaxAngularRate;

    static util::ObservableField<bool> isBlueAlliance;
    static bool isIntakeRunning;
    static bool isAutoTracking;
    static bool initialized;
    static bool isPassing;
    static bool isEndgame;
    static bool wonAuto;
    static bool isOurShift;

    static frc::Translation3d scoreTargetBlue;
    static frc::Translation3d scoreTargetRed;
    static frc::Translation3d passTarget1Blue;
    static frc::Translation3d passTarget2Blue;
    static frc::Translation3d passTarget1Red;
    static frc::Translation3d passTarget2Red;
    static frc::Translation3d target;

    static frc::Pose2d zeroHubPoseBlue;
    static frc::Pose2d zeroHubPoseRed;

    static frc::Timer autoTimer;
    static frc::Timer teleopTimer;

    // Subsystems
    static subsystems::CommandSwerveDrivetrain* drivetrain;
    static subsystems::Vision*          vision;
    static subsystems::Intake*          intake;
    static subsystems::Hopper*          hopper;
    static subsystems::SuperStructure*  superStructure;

    // Shared brake request (used by commands via RobotContainer::brake)
    static swerve::requests::SwerveDriveBrake brake;

    // Named-command helpers
    static frc2::CommandPtr PassLeft();
    static frc2::CommandPtr PassRight();
    static frc2::CommandPtr Score();
    static frc2::CommandPtr StopTracking();
    static frc2::CommandPtr SafeShootAll();
    static frc2::CommandPtr StopSafeShootAll();

    static void UpdateMatchTimers();

    Telemetry m_logger;

private:
    void ConfigureBindings();
    void ConfigAutoBuilder();
    void WarmupAutos();

    // Auto rebuilding
    static void RebuildAutoCommand();
    static frc2::CommandPtr BuildMirroredCommand(
        const wpi::json& commandJson,
        const std::vector<std::shared_ptr<pathplanner::PathPlannerPath>>& paths,
        size_t& pathIndex);

    // Choosers
    static frc::SendableChooser<std::string> autoChooser;
    static frc::SendableChooser<bool>        mirrorChooser;

    // Controller / pedals
    static frc2::CommandPS5Controller controller;
    frc::GenericHID m_pedal{1};
    frc2::Trigger m_leftPedalPressed;
    frc2::Trigger m_middlePedalPressed;
    frc2::Trigger m_rightPedalPressed;

    // NetworkTables
    static nt::NetworkTableInstance s_instance;
    static std::shared_ptr<nt::NetworkTable> s_chooserTable;
    static std::shared_ptr<nt::NetworkTable> s_mirrorTable;

    // Active auto command (owned, never null)
    static frc2::CommandPtr autoCommand;

    // Timer tracking
    static bool autoTimerRunning;
    static bool teleopTimerRunning;
    static bool gsmReceived;

    // Drive requests (stored as members so ApplyRequest lambda can capture by ref)
    swerve::requests::FieldCentric m_drive;
    swerve::requests::PointWheelsAt m_point;

    swerve::requests::ApplyRobotSpeeds m_autoRequest{};
    swerve::requests::FieldCentricFacingAngle m_facingAngleRequest{};
};