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
                               int hoodID, int f1, int f2, int turretID)
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
    turret = new Turret(new hardware::TalonFX{turretID, bus}, m_drivetrain);

    auto rangeCfg = configs::CANrangeConfiguration{}
        .WithFovParams(configs::FovParamsConfigs{}
            .WithFOVRangeX(units::degree_t{6.75})
            .WithFOVRangeY(units::degree_t{6.75}))
        .WithProximityParams(configs::ProximityParamsConfigs{}
            .WithProximityThreshold(units::meter_t{0.02}));
    m_range.GetConfigurator().Apply(rangeCfg);
}

// ── State management ────────────────────────────────────────────────────────
void SuperStructure::BeginTracking()   { m_isAtTarget = false; m_isTracking = true; }
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
void SuperStructure::BeginIntaking()   { m_isIntaking = true; }
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
    // 1. Common Ballistics Math (Runs regardless of state)
    frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        m_drivetrain->GetState().Speeds,
        m_drivetrain->GetState().Pose.Rotation());

    util::BallisticsSolver2::ShotMode mode =
        (RobotContainer::target.ToTranslation2d()
             .Distance(m_drivetrain->GetState().Pose.Translation())
             .value() < 5.0)
        ? util::BallisticsSolver2::ShotMode::HIGHARC
        : util::BallisticsSolver2::ShotMode::LOWARC;

    // Solve for tracking if any shooting mode is active
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

    // 2. Shooter & Turret Control Logic (Independent)
    bool shooterReady = shooter->IsAtTarget();
    bool trackingReady = m_isTracking && solution.valid;
    
    if (m_isOuttaking) {
        shooter->SetExitVelTarget(0.0);
        shooter->GoToTargetHoodAngle(0.0);
        turret->goToTarget(0.0); 
    } else if (trackingReady) {
        shooter->SetExitVelTarget(solution.exitVelocity);
        shooter->GoToTargetHoodAngle(solution.hoodAngle);
        turret->goToTargetFieldRelative(solution.turretAngle);
    } else if (m_isSafeShooting1) {
        shooter->SetExitVelTarget(7.0);
        shooter->GoToTargetHoodAngle(0.0);
        // turret stays where it is or add logic to face forward
    } else if (m_isSpunUp) {
        shooter->GoToTargetSpeed(30.0);
    } else {
        // Idle Shooter
        shooter->SetExitVelTarget(0.0);
        if (!m_isIntaking) shooter->GoToTargetHoodAngle(0.0);
    }

    // 3. Feeder & Intake Interaction Logic (Independent)
    using FC = constants::FeederConstants;
    
    // Logic for the Feeder (the bridge between intake and shooter)
    if (m_isOuttaking) {
        feeder->GoToTargetSpeed(-FC::feederTargetSpeed);
    } else if (trackingReady || m_isSafeShooting1 || m_isSafeShooting2) {
        // Check if shooter is at speed to allow feeding
        if (shooterReady) {
            if (!m_shooterAtSpeedTimer.IsRunning()) m_shooterAtSpeedTimer.Restart();
        } else {
            m_shooterAtSpeedTimer.Stop();
            m_shooterAtSpeedTimer.Reset();
        }
        
        bool readyToShoot = m_shooterAtSpeedTimer.HasElapsed(0.35_s);
        
        // If intaking while shooting, we want the feeder to move only when ready to fire
        // otherwise it holds the ball for the shooter.
        feeder->GoToTargetSpeed(readyToShoot ? FC::feederTargetSpeed : (m_isIntaking ? 2.0 : 0.0));
    } else if (m_isIntaking) {
        // Just intaking, feed balls into the staging area
        feeder->GoToTargetSpeed(2.0); 
        shooter->GoToTargetHoodAngle(0.0); // Safety: stow hood while intaking
    } else {
        feeder->GoToTargetSpeed(0.0);
    }
}
