#include "robot/subsystems/SuperStructure.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include "robot/RobotContainer.h"
#include "robot/Constants.h"
#include "robot/util/Util.h"

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

void SuperStructure::Periodic() {
    using FC = constants::FeederConstants;
    using SC = constants::ShooterConstants;

    const auto driveState = m_drivetrain->GetState();

    const frc::ChassisSpeeds speeds =
        frc::ChassisSpeeds::FromRobotRelativeSpeeds(
            driveState.Speeds,
            driveState.Pose.Rotation());

    const bool inAllianceZone =
        frc::DriverStation::IsTeleopEnabled() &&
        m_drivetrain->IsInAllianceZone();

    const double distanceToTarget =
        RobotContainer::target.ToTranslation2d()
            .Distance(driveState.Pose.Translation())
            .value();

    const auto mode =
        distanceToTarget < 5.5
            ? util::BallisticsSolver2::ShotMode::HIGHARC
            : util::BallisticsSolver2::ShotMode::LOWARC;

    // Update the solver when tracking or safe shooting is requested.
    if (m_isTracking || m_isSafeShooting1 || m_isSafeShooting2) {
        solution = solver.Solve(
            m_drivetrain->GetPositionRelativeField().Translation(),
            frc::Translation3d{
                units::meter_t{speeds.vx.value()},
                units::meter_t{speeds.vy.value()},
                0_m
            },
            frc::Translation3d{},
            RobotContainer::target,
            0.1,
            frc::Translation2d{0.1_m, 0_m},
            mode);
    } else {
        solution = util::BallisticsSolver2::BallisticSolution{};
    }

    frc::SmartDashboard::PutBoolean("Solver/Valid", solution.valid);
    frc::SmartDashboard::PutBoolean("Idle/InAllianceZone", inAllianceZone);

    // Determine if we should be shooting
    const bool activeTrackingShot = m_isTracking && solution.valid;
    const bool activeSafeShot = m_isSafeShooting1 || m_isSafeShooting2;
    const bool shouldShoot = activeTrackingShot || activeSafeShot;
    
    // Check if shooter is at target speed
    const bool shooterAtTarget = shooter->IsAtTarget();
    
    // Track state to detect when shooter FIRST reaches target while shooting
    static bool timerStarted = false;
    
    // Start timer only once when shooter first reaches target during shooting
    if (shouldShoot && shooterAtTarget && !timerStarted) {
        m_shooterAtSpeedTimer.Reset();
        m_shooterAtSpeedTimer.Start();
        timerStarted = true;
    }
    
    // Reset everything when we stop shooting or shooter drops below target
    if (!shouldShoot || !shooterAtTarget) {
        m_shooterAtSpeedTimer.Stop();
        m_shooterAtSpeedTimer.Reset();
        timerStarted = false;
    }
    
    // Feeder runs only after timer has elapsed
    const bool readyToShoot = timerStarted && m_shooterAtSpeedTimer.HasElapsed(0.05_s);
    // Intaking is NOT an active action for the shooter.
    const bool activeAction =
        m_isOuttaking ||
        m_isTracking ||
        m_isSafeShooting1 ||
        m_isSafeShooting2 ||
        m_isSpunUp;

    shooter->SetIdleProfile(!activeAction);

    // --- FEEDER CONTROL ---
    if (m_isIntaking) {
        feeder->GoToTargetSpeed(0.0);
    } else if (m_isOuttaking) {
        feeder->GoToTargetSpeed(-FC::feederTargetSpeed);
    } else if (m_isTracking && solution.valid &&
               std::isfinite(solution.hoodAngle) &&
               std::isfinite(solution.exitVelocity)) {
        if (readyToShoot) {
            feeder->GoToTargetSpeed(FC::feederTargetSpeed);
        } else {
            feeder->GoToTargetSpeed(-2.0);
        }
    } else if (m_isSafeShooting1 || m_isSafeShooting2) {
        if (readyToShoot) {
            feeder->GoToTargetSpeed(FC::feederTargetSpeed);
        } else {
            feeder->GoToTargetSpeed(0.0);
        }
    } else if (m_isSpunUp) {
        feeder->GoToTargetSpeed(-10.0);
    } else {
        feeder->GoToTargetSpeed(0.0);
    }

    // --- SHOOTER CONTROL ---
    if (m_isOuttaking) {
        shooter->SetExitVelTarget(0.0);
        shooter->GoToTargetHoodAngle(0.0);
        shooter->GoToTargetSpeed(0.0);
        m_wasIdle = false;
    }
    else if (m_isTracking) {
        if (!solution.valid || !std::isfinite(solution.hoodAngle) || !std::isfinite(solution.exitVelocity)) {
            shooter->GoToTargetSpeed(0.0);
            shooter->SetExitVelTarget(0.0);
            shooter->GoToTargetHoodAngle(0.0);
            m_wasIdle = false;
        } else {
            shooter->SetExitVelTarget(solution.exitVelocity);
            shooter->GoToTargetHoodAngle(solution.hoodAngle);
            m_wasIdle = false;
        }
    }
    else if (m_isSafeShooting1) {
        shooter->GoToTargetHoodAngle(0.0);
        shooter->SetExitVelTarget(7.0);
        m_wasIdle = false;
    }
    else if (m_isSafeShooting2) {
        shooter->GoToTargetHoodAngle(20.0);
        shooter->SetExitVelTarget(8.0);
        m_wasIdle = false;
    }
    else if (m_isSpunUp) {
        shooter->GoToTargetSpeed(30.0);
        shooter->GoToTargetHoodAngle(0.0);
        m_wasIdle = false;
    }
    // --- IDLE STATE ---
    else {
        m_wasIdle = true;

        if (inAllianceZone) {
            const double idleGoal = SC::idleSpinRps;
            const double rampStep = SC::idleSpinRampRpsPerSec * 0.02;

            m_idleSpinTarget += util::Clamp(
                idleGoal - m_idleSpinTarget,
                -rampStep,
                rampStep
            );
            m_idleSpinTarget = util::Clamp(m_idleSpinTarget, 0.0, idleGoal);

            shooter->GoToTargetSpeed(m_idleSpinTarget);
            shooter->GoToTargetHoodAngle(0.0);

            frc::SmartDashboard::PutNumber("Idle/GoalRPS", idleGoal);
            frc::SmartDashboard::PutNumber("Idle/TargetRPS", m_idleSpinTarget);
        } else {
            m_idleSpinTarget = 0.0;
            shooter->GoToTargetSpeed(0.0);
            shooter->SetExitVelTarget(0.0);
            shooter->GoToTargetHoodAngle(0.0);

            frc::SmartDashboard::PutNumber("Idle/GoalRPS", 0.0);
            frc::SmartDashboard::PutNumber("Idle/TargetRPS", 0.0);
        }
    }
}

frc2::CommandPtr SuperStructure::TrackAndAimCommand() {
    return m_drivetrain->Run([this] {
        if (solution.valid && m_drivetrain->IsInAllianceZone()) {
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