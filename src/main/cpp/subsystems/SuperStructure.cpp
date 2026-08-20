#include "robot/subsystems/SuperStructure.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include "robot/RobotContainer.h"
#include "robot/Constants.h"

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/CANBus.hpp"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include <units/length.h>
#include <units/math.h>
#include <cmath>
#include <numbers>

using namespace subsystems;
using namespace ctre::phoenix6;

SuperStructure::SuperStructure(CommandSwerveDrivetrain* drivetrain,
                               frc2::CommandPS5Controller* controller,
                               int s1, int s2, int s3, int s4,
                               int hoodID, int f1, int f2)
    : m_drivetrain(drivetrain), m_controller(controller)
{
    const CANBus& bus = constants::kSecondaryCanbus;
    auto* m1 = new hardware::TalonFX{s1, bus};
    auto* m2 = new hardware::TalonFX{s2, bus};
    auto* m3 = new hardware::TalonFX{s3, bus};
    auto* m4 = new hardware::TalonFX{s4, bus};
    auto* hood = new hardware::TalonFX{hoodID, bus};
    shooter = new Shooter{m1, m2, m3, m4, hood, drivetrain};
    feeder  = new Feeder{new hardware::TalonFX{f1, bus}, new hardware::TalonFX{f2, bus}};

    auto rangeCfg = configs::CANrangeConfiguration{}
        .WithFovParams(configs::FovParamsConfigs{}
            .WithFOVRangeX(units::degree_t{6.75})
            .WithFOVRangeY(units::degree_t{6.75}))
        .WithProximityParams(configs::ProximityParamsConfigs{}
            .WithProximityThreshold(units::meter_t{0.02}));
    m_range.GetConfigurator().Apply(rangeCfg);

    m_facingAngleRequest.HeadingController.SetPID(10.0, 0.0, 0.4);
    m_facingAngleRequest.HeadingController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
    m_facingAngleRequest.RotationalDeadband = 0_rad_per_s;
}

// ── State management ────────────────────────────────────────────────────────
void SuperStructure::BeginTracking()   { m_isAtTarget = false; m_isTracking = true; m_isIntaking = false; }
void SuperStructure::StopTracking()    { m_isAtTarget = false; m_isTracking = false; }
void SuperStructure::BeginOuttaking()  { m_isOuttaking = true; }
void SuperStructure::StopOuttaking()   { m_isOuttaking = false; }
void SuperStructure::StartPass()       { m_isPassing = true; }
void SuperStructure::StopPass()        { m_isPassing = false; }
void SuperStructure::StartSafeShooting1(){ m_isSafeShooting1 = true; }
void SuperStructure::StopSafeShooting1() { m_isSafeShooting1 = false; }
void SuperStructure::StartSafeShooting2(){ m_isSafeShooting2 = true; }
void SuperStructure::StopSafeShooting2() { m_isSafeShooting2 = false; }
void SuperStructure::StartZoneBased()  { m_isZoneBased = true; }
void SuperStructure::StopZoneBased()   { m_isZoneBased = false; }
void SuperStructure::StartSpunUp()     { m_isSpunUp = true; }
void SuperStructure::StopSpunUp()      { m_isSpunUp = false; }
void SuperStructure::BeginIntaking()   { m_isIntaking = true; m_isTracking = false; }
void SuperStructure::StopIntaking()    { m_isIntaking = false; }

// ── Command factories ────────────────────────────────────────────────────────
frc2::CommandPtr SuperStructure::StartSpunUpCmd()      { return RunOnce([this]{ StartSpunUp(); }); }
frc2::CommandPtr SuperStructure::StopSpunUpCmd()       { return RunOnce([this]{ StopSpunUp(); }); }
frc2::CommandPtr SuperStructure::StartZoneBasedCmd()   { return RunOnce([this]{ StartZoneBased(); }); }
frc2::CommandPtr SuperStructure::StopZoneBasedCmd()    { return RunOnce([this]{ StopZoneBased(); }); }
frc2::CommandPtr SuperStructure::StartSafeShooting1Cmd(){ return RunOnce([this]{ StartSafeShooting1(); }); }
frc2::CommandPtr SuperStructure::StopSafeShooting1Cmd() { return RunOnce([this]{ StopSafeShooting1(); }); }
frc2::CommandPtr SuperStructure::StartSafeShooting2Cmd(){ return RunOnce([this]{ StartSafeShooting2(); }); }
frc2::CommandPtr SuperStructure::StopSafeShooting2Cmd() { return RunOnce([this]{ StopSafeShooting2(); }); }
frc2::CommandPtr SuperStructure::StartPassingCmd()     { return RunOnce([this]{ StartPass(); }); }
frc2::CommandPtr SuperStructure::StopPassingCmd()      { return RunOnce([this]{ StopPass(); }); }
frc2::CommandPtr SuperStructure::StartTrackingCmd()    { return RunOnce([this]{ BeginTracking(); }); }
frc2::CommandPtr SuperStructure::StopTrackingCmd()     { return RunOnce([this]{ StopTracking(); }); }
frc2::CommandPtr SuperStructure::StartOuttakingCmd()   { return RunOnce([this]{ BeginOuttaking(); }); }
frc2::CommandPtr SuperStructure::StopOuttakingCmd()    { return RunOnce([this]{ StopOuttaking(); }); }
frc2::CommandPtr SuperStructure::StartIntakingCmd()    { return RunOnce([this]{ BeginIntaking(); }); }
frc2::CommandPtr SuperStructure::StopIntakingCmd()     { return RunOnce([this]{ StopIntaking(); }); }
frc2::CommandPtr SuperStructure::SwitchModes()         { return RunOnce([this]{ m_isDefenseMode = !m_isDefenseMode; }); }

// ── Periodic ─────────────────────────────────────────────────────────────────
void SuperStructure::Periodic() {
    bool readyToShoot = false;

    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        m_drivetrain->GetState().Speeds,
        m_drivetrain->GetState().Pose.Rotation());

    util::BallisticsSolver2::ShotMode mode =
        (RobotContainer::target.ToTranslation2d()
             .Distance(m_drivetrain->GetState().Pose.Translation())
             .value() < 5.5)
        ? util::BallisticsSolver2::ShotMode::HIGHARC
        : util::BallisticsSolver2::ShotMode::LOWARC;

    bool shooterReady  = shooter->IsAtTarget();
    bool trackingReady = m_isTracking && solution.valid;
    bool safeReady     = m_isSafeShooting1 || m_isSafeShooting2;

    if ((trackingReady || safeReady) && shooterReady) {
        if (!m_shooterAtSpeedTimer.IsRunning()) m_shooterAtSpeedTimer.Restart();
    } else {
        m_shooterAtSpeedTimer.Stop();
        m_shooterAtSpeedTimer.Reset();
    }
    readyToShoot = m_shooterAtSpeedTimer.HasElapsed(0.35_s);

    if (m_isTracking || m_isSafeShooting1 || m_isSafeShooting2) {
        solution = solver.Solve(
            m_drivetrain->GetPositionRelativeField().Translation(),
            frc::Translation3d{units::meter_t{speeds.vx.value()},
                               units::meter_t{speeds.vy.value()},
                               0_m},
            frc::Translation3d{},
            RobotContainer::target,
            0.1,
            frc::Translation2d{0.1_m, 0_m},
            mode);
    } else {
        solution = util::BallisticsSolver2::BallisticSolution{};
    }

    frc::SmartDashboard::PutBoolean("Solver/Valid",         solution.valid);
    frc::SmartDashboard::PutNumber ("Solver/Hood",          solution.hoodAngle);
    frc::SmartDashboard::PutNumber ("Solver/ExitVel",       solution.exitVelocity);
    frc::SmartDashboard::PutNumber ("Solver/TurretAngle",   solution.turretAngle);
    frc::SmartDashboard::PutNumber ("Solver/TOF",           solution.timeOfFlight);
    frc::SmartDashboard::PutNumberArray("Target",
        std::vector<double>{RobotContainer::target.X().value(),
                            RobotContainer::target.Y().value(),
                            RobotContainer::target.Z().value(), 0.0});

    using FC = constants::FeederConstants;

    if (m_isOuttaking) {
        shooter->SetExitVelTarget(0.0);
        shooter->GoToTargetHoodAngle(0.0);
        feeder->GoToTargetSpeed(-FC::feederTargetSpeed);
    } else if (m_isIntaking) {
        shooter->GoToTargetHoodAngle(0.0);
    } else if (m_isTracking && solution.valid) {
        if (!std::isfinite(solution.hoodAngle)) return;
        shooter->SetExitVelTarget(solution.exitVelocity);
        shooter->GoToTargetHoodAngle(solution.hoodAngle);
        feeder->GoToTargetSpeed(readyToShoot ? FC::feederTargetSpeed : -2.0);
    } else if (m_isSafeShooting1) {
        shooter->GoToTargetHoodAngle(0.0);
        shooter->SetExitVelTarget(7.0);
        feeder->GoToTargetSpeed(readyToShoot ? FC::feederTargetSpeed : 0.0);
    } else if (m_isSafeShooting2) {
        shooter->GoToTargetHoodAngle(20.0);
        shooter->SetExitVelTarget(8.0);
        feeder->GoToTargetSpeed(readyToShoot ? FC::feederTargetSpeed : 0.0);
    } else if (m_isSpunUp) {
        shooter->GoToTargetSpeed(30.0);
        shooter->GoToTargetHoodAngle(0.0);
        feeder->GoToTargetSpeed(-10.0);
    } else {
        shooter->SetExitVelTarget(0.0);
        shooter->GoToTargetHoodAngle(0.0);
        feeder->GoToTargetSpeed(0.0);
    }
}

frc2::CommandPtr SuperStructure::TrackAndAimCommand() {
    return m_drivetrain->Run([this] {
        if (solution.valid) {
            double targetDeg = solution.turretAngle + 
                               (RobotContainer::isBlueAlliance.GetValue() ? 0.0 : 180.0);
            double currentDeg = m_drivetrain->GetState().Pose.Rotation().Degrees().value() + 
                                (RobotContainer::isBlueAlliance.GetValue() ? 0.0 : 180.0);
            double angleErr = std::abs(frc::InputModulus(targetDeg - currentDeg, -180.0, 180.0));

            frc::SmartDashboard::PutBoolean("Defense mode", m_isDefenseMode);
            frc::SmartDashboard::PutNumber("Angle error", angleErr);

            double maxSpd = RobotContainer::MaxSpeed;

            double mag = std::hypot(m_controller->GetLeftY(), m_controller->GetLeftX());

            if (mag < 0.2 && m_isDefenseMode && angleErr < 2.0) {
                m_drivetrain->SetControl(
                    m_pointRequest.WithModuleDirection(frc::Rotation2d{0_deg})
                );
            } else {
                double targetDegWrapped = frc::InputModulus(targetDeg, -180.0, 180.0);
                m_drivetrain->SetControl(
                    m_facingAngleRequest
                        .WithTargetDirection(frc::Rotation2d{units::degree_t{targetDegWrapped}})
                        .WithVelocityX(units::meters_per_second_t{-m_controller->GetLeftY() * maxSpd / 2})
                        .WithVelocityY(units::meters_per_second_t{-m_controller->GetLeftX() * maxSpd / 2})
                        .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::Velocity)
                );
            }
        } else {
            m_drivetrain->SetControl(
                m_fieldCentricRequest
                    .WithVelocityX(units::meters_per_second_t{-m_controller->GetLeftY() * RobotContainer::MaxSpeed})
                    .WithVelocityY(units::meters_per_second_t{-m_controller->GetLeftX() * RobotContainer::MaxSpeed})
                    .WithRotationalRate(units::radians_per_second_t{-m_controller->GetRightX() * RobotContainer::MaxAngularRate})
                    .WithDeadband(units::meters_per_second_t{0.1 * RobotContainer::MaxSpeed})
                    .WithRotationalDeadband(units::radians_per_second_t{0.1 * RobotContainer::MaxAngularRate})
                    .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::Velocity)
            );
        }
    })
    .BeforeStarting([this] { BeginTracking(); })
    .FinallyDo([this] { StopTracking(); })
    .WithName("StickyTrackAndAim");
}

frc2::CommandPtr SuperStructure::SafeShotCommand1() {
    auto cmd = this->Run([this] {
        StartSafeShooting1();
        m_drivetrain->SetControl(RobotContainer::brake);
    }).FinallyDo([this] { StopSafeShooting1(); });
    cmd.get()->AddRequirements(m_drivetrain);
    return cmd;
}

frc2::CommandPtr SuperStructure::SafeShotCommand2() {
    auto cmd = this->Run([this] {
        StartSafeShooting2();
        m_drivetrain->SetControl(RobotContainer::brake);
    }).FinallyDo([this] { StopSafeShooting2(); });
    cmd.get()->AddRequirements(m_drivetrain);
    return cmd;
}
