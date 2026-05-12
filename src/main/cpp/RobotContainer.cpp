#include "robot/RobotContainer.h"
#include "robot/Constants.h"

#include "ctre/phoenix6/swerve/SwerveRequest.hpp"
#include "ctre/phoenix6/Utils.hpp"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/FollowPathCommand.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/config/PIDConstants.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/path/PathPlannerPath.h>

#include <frc/DriverStation.h>
#include <frc/Filesystem.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>

#include <units/angle.h>
#include <units/length.h>
#include <numbers>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <wpi/json.h>

using namespace ctre::phoenix6;

// ── Static member definitions ────────────────────────────────────────────────
double   RobotContainer::MaxSpeed    = TunerConstants::kSpeedAt12Volts.value();
double   RobotContainer::MaxAngularRate = 0.75 * 2.0 * std::numbers::pi; // 3/4 rot/s in rad/s

util::ObservableField<bool> RobotContainer::isBlueAlliance{true};
bool RobotContainer::isIntakeRunning  = false;
bool RobotContainer::isAutoTracking   = false;
bool RobotContainer::initialized      = false;
bool RobotContainer::isPassing        = false;
bool RobotContainer::isEndgame        = false;
bool RobotContainer::wonAuto          = false;
bool RobotContainer::isOurShift       = false;

frc::Translation3d RobotContainer::scoreTargetBlue{4.6470_m, 4.0228_m, 1.2_m};
frc::Translation3d RobotContainer::scoreTargetRed {11.8882_m, 4.0228_m, 1.2_m};
frc::Translation3d RobotContainer::passTarget1Blue{1.5_m, 6.5_m, 0_m};
frc::Translation3d RobotContainer::passTarget2Blue{1.5_m, 1.5_m, 0_m};
frc::Translation3d RobotContainer::passTarget1Red {14.5_m, 6.5_m, 0_m};
frc::Translation3d RobotContainer::passTarget2Red {14.5_m, 1.5_m, 0_m};
frc::Translation3d RobotContainer::target = RobotContainer::scoreTargetBlue;

frc::Pose2d RobotContainer::zeroHubPoseBlue{
    frc::Translation2d{units::inch_t{150}, units::inch_t{158.6}},
    frc::Rotation2d{0_deg}};
frc::Pose2d RobotContainer::zeroHubPoseRed{
    frc::Translation2d{units::inch_t{503.6}, units::inch_t{158.6}},
    frc::Rotation2d{180_deg}};

frc::Timer RobotContainer::autoTimer{};
frc::Timer RobotContainer::teleopTimer{};

subsystems::CommandSwerveDrivetrain* RobotContainer::drivetrain  = nullptr;
subsystems::Vision*          RobotContainer::vision         = nullptr;
subsystems::Intake*          RobotContainer::intake         = nullptr;
subsystems::Hopper*          RobotContainer::hopper         = nullptr;
subsystems::SuperStructure*  RobotContainer::superStructure = nullptr;

swerve::requests::SwerveDriveBrake RobotContainer::brake{};

frc::SendableChooser<std::string> RobotContainer::autoChooser{};
frc::SendableChooser<bool>        RobotContainer::mirrorChooser{};
frc2::CommandPS5Controller        RobotContainer::controller{0};

nt::NetworkTableInstance              RobotContainer::s_instance  = nt::NetworkTableInstance::GetDefault();
std::shared_ptr<nt::NetworkTable>     RobotContainer::s_chooserTable = s_instance.GetTable("SmartDashboard")->GetSubTable("Auto Chooser");
std::shared_ptr<nt::NetworkTable>     RobotContainer::s_mirrorTable  = s_instance.GetTable("SmartDashboard")->GetSubTable("Mirror Path");

frc2::CommandPtr RobotContainer::autoCommand = frc2::cmd::None();
bool RobotContainer::autoTimerRunning   = false;
bool RobotContainer::teleopTimerRunning = false;
bool RobotContainer::gsmReceived        = false;

// ── Constructor ───────────────────────────────────────────────────────────────
RobotContainer::RobotContainer()
    : m_leftPedalPressed{[this]{ return m_pedal.GetRawButton(1); }},
      m_middlePedalPressed{[this]{ return m_pedal.GetRawButton(2); }},
      m_rightPedalPressed{[this]{ return m_pedal.GetRawButton(3); }},
      m_logger{units::meters_per_second_t{TunerConstants::kSpeedAt12Volts.value()}}
{
    // Allocate subsystems on heap (owned for robot lifetime)
    static subsystems::CommandSwerveDrivetrain dt = TunerConstants::CreateDrivetrain();
    drivetrain = &dt;

    static subsystems::SuperStructure ss{drivetrain, &controller, 50, 52, 51, 53, 45, 5, 62, 0};
    superStructure = &ss;

    static subsystems::Vision vis{drivetrain, ss.turret};
    vision = &vis;

    static subsystems::Intake intk{drivetrain};
    intake = &intk;

    static subsystems::Hopper hop{};
    hopper = &hop;

    // Register named commands for PathPlanner
    pathplanner::NamedCommands::registerCommand("RunIntake",      intake->Run().Unwrap());
    pathplanner::NamedCommands::registerCommand("StopIntake",     intake->Stop().Unwrap());
    pathplanner::NamedCommands::registerCommand("RunHopper",      hopper->Run().Unwrap());
    pathplanner::NamedCommands::registerCommand("StopHopper",     hopper->Stop().Unwrap());
    pathplanner::NamedCommands::registerCommand("IntakeDown",
        intake->MoveDown().WithTimeout(0.3_s).Unwrap());
    pathplanner::NamedCommands::registerCommand("IntakeUp",       intake->Stow().Unwrap());
    pathplanner::NamedCommands::registerCommand("IntakeBumpAngle",intake->AngleForBump().Unwrap());
    pathplanner::NamedCommands::registerCommand("BeginTracking",    superStructure->StartTrackingCmd());
    pathplanner::NamedCommands::registerCommand("StopTracking",    superStructure->StopTrackingCmd());
    pathplanner::NamedCommands::registerCommand("SlowPush",       intake->SlowPush().Unwrap());
    pathplanner::NamedCommands::registerCommand("StartSpinUp",    superStructure->StartSpunUpCmd().Unwrap());
    pathplanner::NamedCommands::registerCommand("StopSpinUp",     superStructure->StopSpunUpCmd().Unwrap());

    // Warm up solver
    superStructure->solver.Solve(
        frc::Translation3d{}, frc::Translation3d{}, frc::Translation3d{},
        frc::Translation3d{}, 0.1,
        frc::Translation2d{},
        util::BallisticsSolver2::ShotMode::HIGHARC);

    drivetrain->ResetRotation(
        frc::Rotation2d{isBlueAlliance.GetValue() ? 0_deg : 180_deg});

    // Populate auto chooser
    auto autoNames = pathplanner::AutoBuilder::getAllAutoNames();
    if (!autoNames.empty()) {
        autoChooser.SetDefaultOption(autoNames[0], autoNames[0]);
        for (const auto& name : autoNames) autoChooser.AddOption(name, name);
    }
    autoChooser.AddOption("None", "");
    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

    mirrorChooser.SetDefaultOption("Normal", false);
    mirrorChooser.AddOption("Mirrored", true);
    frc::SmartDashboard::PutData("Mirror Path", &mirrorChooser);

    // Listen for changes to auto selection, mirror selection, and alliance colour
    s_instance.AddListener(
        s_chooserTable->GetEntry("active").GetTopic(),
        nt::EventFlags::kValueAll,
        [](const nt::Event&) noexcept {
            try { RebuildAutoCommand(); }
            catch (...) { std::fprintf(stderr, "Exception in chooser listener\n"); }
        }
    );

    s_instance.AddListener(
        s_mirrorTable->GetEntry("active").GetTopic(),
        nt::EventFlags::kValueAll,
        [](const nt::Event&) noexcept {
            try { RebuildAutoCommand(); }
            catch (...) { std::fprintf(stderr, "Exception in mirror listener\n"); }
        }
    );

    isBlueAlliance.AddListener([](bool, bool) noexcept {
        try { RebuildAutoCommand(); }
        catch (...) { std::fprintf(stderr, "Exception in alliance listener\n"); }
    });

    ConfigAutoBuilder();
    ConfigureBindings();
    WarmupAutos();

    try {
        RebuildAutoCommand();
    } catch (...) {
        std::fprintf(stderr, "[ctor] RebuildAutoCommand failed\n");
        autoCommand = frc2::cmd::None();
    }
}

// ── Rebuild Auto Command ─────────────────────────────────────────────────────
void RobotContainer::RebuildAutoCommand() {
    try {
    if (!initialized) return;
    std::string autoName = autoChooser.GetSelected();
    if (autoName.empty() || autoName == "None") {
        autoCommand = frc2::cmd::None();
        return;
    }
    if (mirrorChooser.GetSelected()) {
        try {
            auto deployPath = std::filesystem::path(frc::filesystem::GetDeployDirectory()) / "pathplanner" / "autos" / (autoName + ".auto");
            std::ifstream f(deployPath);
            if (!f.good()) {
                autoCommand = frc2::cmd::None();
                return;
            }
            std::string jsonStr((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            wpi::json autoJson = wpi::json::parse(jsonStr);
            const wpi::json& commandJson = autoJson.at("command");
            
            auto paths = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(autoName);
            size_t pathIndex = 0;

            if (!paths.empty()) {
                // 1. Get the first path and mirror it (Left/Right flip)
                auto startPath = paths[0]->mirrorPath();
                
                // 2. Flip it to the Red side if we are not on Blue Alliance
                if (!isBlueAlliance.GetValue()) {
                    startPath = startPath->flipPath();
                }

                // 3. Extract the starting pose and seed the drivetrain
                auto startPose = startPath->getStartingHolonomicPose();
                if (startPose.has_value()) {
                    drivetrain->ResetPose(startPose.value());
                }
            }

            // Generate the command tree using the transformed path logic
            autoCommand = BuildMirroredCommand(commandJson, paths, pathIndex);
            
        } catch (std::exception& e) {
            std::fprintf(stderr, "Error rebuilding mirrored auto: %s\n", e.what());
            autoCommand = frc2::cmd::None();
        }
    } else {
        try {
            autoCommand = pathplanner::PathPlannerAuto(autoName).ToPtr();
        } catch (std::exception& e) {
            autoCommand = frc2::cmd::None();
        }
    }
    } catch (std::exception& e) {
        std::fprintf(stderr, "[RebuildAutoCommand] Exception: %s\n", e.what());
        autoCommand = frc2::cmd::None();
    } catch (...) {
        std::fprintf(stderr, "[RebuildAutoCommand] Unknown exception\n");
        autoCommand = frc2::cmd::None();
    }
}

frc2::CommandPtr RobotContainer::BuildMirroredCommand(
        const wpi::json& commandJson,
        const std::vector<std::shared_ptr<pathplanner::PathPlannerPath>>& paths,
        size_t& pathIndex) {
    if (commandJson.empty() || !commandJson.contains("type")) return frc2::cmd::None();
    std::string type = commandJson["type"].get<std::string>();
    const wpi::json& data = commandJson["data"];

    // Recursion Handling (Standard structural logic)
    if (type == "sequential" || type == "parallel" || type == "race") {
        std::vector<frc2::CommandPtr> commands;
        for (const auto& child : data["commands"]) {
            commands.emplace_back(BuildMirroredCommand(child, paths, pathIndex));
        }
        if (type == "sequential") return frc2::cmd::Sequence(std::move(commands));
        if (type == "parallel")   return frc2::cmd::Parallel(std::move(commands));
        if (type == "race")       return frc2::cmd::Race(std::move(commands));
    } else if (type == "deadline") {
        std::vector<frc2::CommandPtr> commands;
        for (const auto& child : data["commands"]) {
            commands.emplace_back(BuildMirroredCommand(child, paths, pathIndex));
        }
        if (commands.empty()) return frc2::cmd::None();
        auto deadlineCmd = std::move(commands[0]);
        commands.erase(commands.begin());
        return frc2::cmd::Deadline(std::move(deadlineCmd), std::move(commands));
    } 
    
    // Path Following Logic
    else if (type == "path") {
        if (pathIndex < paths.size()) {
            auto pathPtr = paths[pathIndex++];
            auto mirroredPath = pathPtr->mirrorPath();
            if (!RobotContainer::isBlueAlliance.GetValue()) {
                mirroredPath = mirroredPath->flipPath();
            }
            try {
                auto config = pathplanner::RobotConfig::fromGUISettings();
                auto holonomicController = std::make_shared<pathplanner::PPHolonomicDriveController>(
                    pathplanner::PIDConstants{5.0, 0.0, 0.0},
                    pathplanner::PIDConstants{5.0, 0.0, 0.0}
                );

                return frc2::CommandPtr(std::make_unique<pathplanner::FollowPathCommand>(
                    mirroredPath,
                    // Safety check on drivetrain pointer
                    []() { 
                        return RobotContainer::drivetrain ? RobotContainer::drivetrain->GetState().Pose : frc::Pose2d{}; 
                    },
                    []() { 
                        return RobotContainer::drivetrain ? RobotContainer::drivetrain->GetState().Speeds : frc::ChassisSpeeds{}; 
                    },
                    [](frc::ChassisSpeeds speeds, pathplanner::DriveFeedforwards ff) {
                        if (!RobotContainer::drivetrain) return;

                        // FIX: Use a static request object to prevent EXC_BAD_ACCESS
                        // This ensures the memory address remains valid for the drivetrain thread.
                        static swerve::requests::ApplyRobotSpeeds pathRequest{};

                        RobotContainer::drivetrain->SetControl(
                            pathRequest.WithSpeeds(speeds)
                                .WithWheelForceFeedforwardsX(ff.robotRelativeForcesX)
                                .WithWheelForceFeedforwardsY(ff.robotRelativeForcesY)
                                .WithDriveRequestType(swerve::DriveRequestType::Velocity)
                        );
                    },
                    holonomicController,
                    config,
                    []() { return false; }, 
                    frc2::Requirements{RobotContainer::drivetrain} 
                ));
            } catch (std::exception& e) {
                return frc2::cmd::None();
            }
        }
    } else if (type == "named") {
        return pathplanner::NamedCommands::getCommand(data["name"].get<std::string>());
    } else if (type == "wait") {
        return frc2::cmd::Wait(units::second_t{data["waitTime"].get<double>()});
    }
    return frc2::cmd::None();
}

// ── Bindings and AutoBuilder config ──────────────────────────────────────────
void RobotContainer::ConfigureBindings() {
    // Default drive command
    drivetrain->SetDefaultCommand(drivetrain->ApplyRequest([this]() {
        return m_drive
            .WithVelocityX(units::meters_per_second_t{-controller.GetLeftY() * MaxSpeed})
            .WithVelocityY(units::meters_per_second_t{-controller.GetLeftX() * MaxSpeed})
            .WithRotationalRate(units::radians_per_second_t{-controller.GetRightX() * MaxAngularRate});
    }));
    drivetrain->RegisterTelemetry([this](auto const& state){ m_logger.Telemeterize(state); });

    controller.R3().OnTrue(intake->AngleForBump());
    controller.Options().OnTrue(superStructure->SwitchModes());
    controller.Cross().WhileTrue(drivetrain->ApplyRequest([this]() -> auto&& {
        return brake;
    }));
    controller.L1().OnTrue(frc2::cmd::Either(
        superStructure->StopTrackingCmd().AlongWith(hopper->Stop()), 
        superStructure->StartTrackingCmd().AlongWith(hopper->Run()),
        [this] { return superStructure->IsTracking(); }
    ));

    controller.R1().OnTrue(frc2::cmd::Either(
        superStructure->StopIntakingCmd()
            .AlongWith(intake->Stow().AndThen(intake->Stop())),
        superStructure->StartIntakingCmd()
            .AlongWith(intake->MoveDown().AndThen(intake->Run())),
        [this] { return superStructure->IsIntaking(); }
    ));
    controller.POVLeft().OnTrue(intake->MoveDown());
    controller.POVRight().OnTrue(
        intake->Stow()
        .AndThen(intake->Stop())
        .AndThen(hopper->Stop())
        .AndThen(superStructure->StopIntakingCmd()));
    controller.POVUp().OnTrue(intake->AutoZero());
    controller.Circle()
        .OnTrue(superStructure->StartOuttakingCmd()
            .AndThen(hopper->Outtake())
            .AndThen(intake->Outtake()))
        .OnFalse(superStructure->StopOuttakingCmd()
            .AndThen(hopper->Stop())
            .AndThen(intake->Stop()));
    controller.POVDown()
        .OnTrue(superStructure->StartZoneBasedCmd()
            .AlongWith(hopper->Run())
            .AlongWith(intake->SlowPush()))
        .OnFalse(superStructure->StopZoneBasedCmd()
            .AlongWith(intake->MoveDown()));

    controller.Create().OnTrue(
        drivetrain->RunOnce([this]{ drivetrain->SeedFieldCentric(); })
        .AndThen(vision->RunOnce([this]{ vision->StopIMUData(); }))
        .AndThen(vision->RunOnce([this]{ vision->ResetOrientation(); }))
        .AndThen(drivetrain->RunOnce([this]{
            drivetrain->ResetPose(
                isBlueAlliance.GetValue() ? zeroHubPoseBlue : zeroHubPoseRed);
        })));
}

void RobotContainer::ConfigAutoBuilder() {
    try {
        auto config = pathplanner::RobotConfig::fromGUISettings();
        pathplanner::AutoBuilder::configure(
            [this]{ return drivetrain->GetState().Pose; },
            [this](frc::Pose2d p){ drivetrain->ResetPose(p); },
            [this]{ return drivetrain->GetState().Speeds; },
            [this](frc::ChassisSpeeds speeds, pathplanner::DriveFeedforwards ff){
                drivetrain->SetControl(
                    m_autoRequest
                        .WithSpeeds(speeds)
                        .WithWheelForceFeedforwardsX(ff.robotRelativeForcesX)
                        .WithWheelForceFeedforwardsY(ff.robotRelativeForcesY)
                        .WithDriveRequestType(swerve::DriveRequestType::Velocity));
            },
            std::make_shared<pathplanner::PPHolonomicDriveController>(
                pathplanner::PIDConstants{5, 0, 0},
                pathplanner::PIDConstants{5, 0, 0}),
            config,
            []{ return !isBlueAlliance.GetValue(); },
            drivetrain);

        pathplanner::PPHolonomicDriveController::setRotationTargetOverride(
            []() -> std::optional<frc::Rotation2d> {
                if (isAutoTracking && superStructure->solution.valid) {
                    double deg = superStructure->solution.turretAngle +
                                 (isBlueAlliance.GetValue() ? 0.0 : 180.0);
                    return frc::Rotation2d{units::degree_t{deg}};
                }
                return std::nullopt;
            });
    } catch (std::exception const& ex) {
    std::fprintf(stderr, "Failed to configure AutoBuilder: %s\n", ex.what());
    } catch (...) {
        std::fprintf(stderr, "Failed to configure AutoBuilder: unknown exception\n");
    }
}

void RobotContainer::WarmupAutos() {
    for (const auto& name : pathplanner::AutoBuilder::getAllAutoNames()) {
        try { auto autoCommand = pathplanner::PathPlannerAuto(name).ToPtr(); } catch (...) {}
    }
    drivetrain->SetControl(swerve::requests::Idle{});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return autoCommand.get();
}

// ── Static command helpers ────────────────────────────────────────────────────
frc2::CommandPtr RobotContainer::PassLeft() {
    return frc2::cmd::RunOnce([]{ isPassing = true;
        target = isBlueAlliance.GetValue() ? passTarget1Blue : passTarget1Red; });
}
frc2::CommandPtr RobotContainer::PassRight() {
    return frc2::cmd::RunOnce([]{ isPassing = true;
        target = isBlueAlliance.GetValue() ? passTarget2Blue : passTarget2Red; });
}
frc2::CommandPtr RobotContainer::Score() {
    return frc2::cmd::RunOnce([]{ isPassing = false;
        target = isBlueAlliance.GetValue() ? scoreTargetBlue : scoreTargetRed; });
}
frc2::CommandPtr RobotContainer::StopTracking()   { return superStructure->StopTrackingCmd(); }
frc2::CommandPtr RobotContainer::SafeShootAll()   { return superStructure->StartSafeShooting1Cmd(); }
frc2::CommandPtr RobotContainer::StopSafeShootAll(){ return superStructure->StopSafeShooting1Cmd(); }

void RobotContainer::UpdateMatchTimers() {
    bool isAuto   = frc::DriverStation::IsAutonomousEnabled();
    bool isTeleop = frc::DriverStation::IsTeleopEnabled();

    if (isAuto) {
        if (!autoTimerRunning) { autoTimer.Reset(); autoTimer.Start(); autoTimerRunning = true; }
    } else if (autoTimerRunning) {
        autoTimer.Stop(); autoTimerRunning = false;
    }

    if (isTeleop) {
        if (!teleopTimerRunning) { teleopTimer.Reset(); teleopTimer.Start(); teleopTimerRunning = true; }

        if (!gsmReceived) {
            std::string gsm = frc::DriverStation::GetGameSpecificMessage();
            if (!gsm.empty()) {
                char first = std::toupper(gsm[0]);
                wonAuto = (first == 'B' && isBlueAlliance.GetValue()) ||
                          (first == 'R' && !isBlueAlliance.GetValue());
                gsmReceived = true;
            }
        }

        double t = teleopTimer.Get().value();
        isEndgame = t >= 110.0;

        if (t < 10.0 || isEndgame)   isOurShift = true;
        else if (t < 35.0)           isOurShift = !wonAuto;
        else if (t < 60.0)           isOurShift =  wonAuto;
        else if (t < 85.0)           isOurShift = !wonAuto;
        else                         isOurShift =  wonAuto;
    } else if (teleopTimerRunning) {
        teleopTimer.Stop(); teleopTimerRunning = false;
        isEndgame = false; isOurShift = false; wonAuto = false; gsmReceived = false;
    }

    frc::SmartDashboard::PutNumber ("Auto Timer",    autoTimer.Get().value());
    frc::SmartDashboard::PutNumber ("Teleop Timer",  teleopTimer.Get().value());
    frc::SmartDashboard::PutBoolean("Is Endgame",    isEndgame);
    frc::SmartDashboard::PutBoolean("Is Our Shift",  isOurShift);
    frc::SmartDashboard::PutBoolean("Won Auto",      wonAuto);
}