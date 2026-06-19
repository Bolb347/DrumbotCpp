#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/filter/Debouncer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"

#include <units/angular_velocity.h>
#include <units/math.h>

// Forward declaration
namespace subsystems { class CommandSwerveDrivetrain; }

namespace subsystems {

class Shooter : public frc2::SubsystemBase {
public:
    Shooter(ctre::phoenix6::hardware::TalonFX* motor1,
            ctre::phoenix6::hardware::TalonFX* motor2,
            ctre::phoenix6::hardware::TalonFX* motor3,
            ctre::phoenix6::hardware::TalonFX* motor4,
            ctre::phoenix6::hardware::TalonFX* hood,
            CommandSwerveDrivetrain* drivetrain);

    double Rps();
    void GoToTargetSpeed(double targetSpeed);
    void GoToTargetHoodAngle(double targetAngleDegFromVertical);
    void GoToTargetRot(double targetRot);
    void SetExitVelTarget(double targetExitVelocity);
    bool IsAtTarget();

    void Periodic() override;

private:
    ctre::phoenix6::hardware::TalonFX* m_motor1;
    ctre::phoenix6::hardware::TalonFX* m_motor2;
    ctre::phoenix6::hardware::TalonFX* m_motor3;
    ctre::phoenix6::hardware::TalonFX* m_motor4;
    ctre::phoenix6::hardware::TalonFX* m_hood;
    CommandSwerveDrivetrain* m_drivetrain;

    ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_vel1;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_vel2;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_vel3;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> m_vel4;

    double m_currTarget = 0.0;
    frc::Debouncer m_stableDebouncer{0.4_s, frc::Debouncer::DebounceType::kBoth};
};

} // namespace subsystems
