#pragma once
#include <array>
#include <frc/Timer.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/CommandPS5Controller.h>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Feeder.h"
#include "Shooter.h"
#include "Turret.h"
#include "CommandSwerveDrivetrain.h"
#include "robot/util/BallisticsSolver2.h"

namespace subsystems {

class SuperStructure : public frc2::SubsystemBase {
public:
    SuperStructure(CommandSwerveDrivetrain* drivetrain,
                   frc2::CommandPS5Controller* controller,
                   int shooterMotorID1, int shooterMotorID2,
                   int shooterMotorID3, int shooterMotorID4,
                   int hoodMotorID,
                   int feederMotorID1, int feederMotorID2,
                   int turretMotorID);

    // State setters
    void BeginTracking();
    void StopTracking();
    void BeginOuttaking();
    void StopOuttaking();
    void StartPass();
    void StopPass();
    void StartSafeShooting1();
    void StopSafeShooting1();
    void StartSafeShooting2();
    void StopSafeShooting2();
    void StartZoneBased();
    void StopZoneBased();
    void StartSpunUp();
    void StopSpunUp();
    void BeginIntaking();
    void StopIntaking();

    bool IsIntaking() {
        return m_isIntaking;
    }

    bool IsTracking() {
        return m_isTracking;
    }

    // Command factories
    frc2::CommandPtr StartSpunUpCmd();
    frc2::CommandPtr StopSpunUpCmd();
    frc2::CommandPtr StartZoneBasedCmd();
    frc2::CommandPtr StopZoneBasedCmd();
    frc2::CommandPtr StartSafeShooting1Cmd();
    frc2::CommandPtr StopSafeShooting1Cmd();
    frc2::CommandPtr StartSafeShooting2Cmd();
    frc2::CommandPtr StopSafeShooting2Cmd();
    frc2::CommandPtr StartPassingCmd();
    frc2::CommandPtr StopPassingCmd();
    frc2::CommandPtr StartTrackingCmd();
    frc2::CommandPtr StopTrackingCmd();
    frc2::CommandPtr StartOuttakingCmd();
    frc2::CommandPtr StopOuttakingCmd();
    frc2::CommandPtr StartIntakingCmd();
    frc2::CommandPtr StopIntakingCmd();
    frc2::CommandPtr SwitchModes();

    void Periodic() override;

    util::BallisticsSolver2::BallisticSolution solution;
    util::BallisticsSolver2 solver;

    Shooter* shooter;
    Feeder*  feeder;
    Turret* turret;

private:
    CommandSwerveDrivetrain* m_drivetrain;
    frc2::CommandPS5Controller* m_controller;

    swerve::requests::FieldCentricFacingAngle m_facingAngleRequest{};
    swerve::requests::FieldCentric            m_fieldCentricRequest{};
    swerve::requests::PointWheelsAt           m_pointRequest{};

    bool m_isTracking      = false;
    bool m_isOuttaking     = false;
    bool m_isPassing       = false;
    bool m_isSafeShooting1 = false;
    bool m_isSafeShooting2 = false;
    bool m_isZoneBased     = false;
    bool m_isAtTarget      = false;
    bool m_isSpunUp        = false;
    bool m_isIntaking      = false;
    bool m_isDefenseMode   = false;

    ctre::phoenix6::hardware::CANrange m_range{60, "team3045-2"};
    frc::Timer m_shooterAtSpeedTimer;
};

} // namespace subsystems
