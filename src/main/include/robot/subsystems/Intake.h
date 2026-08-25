#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

// Forward declaration
namespace subsystems { class CommandSwerveDrivetrain; }

namespace subsystems {

class Intake : public frc2::SubsystemBase {
public:
    enum class IntakeState { INTAKING, OUTTAKING, STOPPED };

    explicit Intake(CommandSwerveDrivetrain* drivetrain);

    frc2::CommandPtr Run();
    frc2::CommandPtr Outtake();
    frc2::CommandPtr Stop();
    frc2::CommandPtr ToggleIntake();
    frc2::CommandPtr MoveDown();
    frc2::CommandPtr Stow();
    frc2::CommandPtr AngleForBump();
    frc2::CommandPtr SlowPush();
    frc2::CommandPtr AutoZero();

    void Periodic() override;

    IntakeState state = IntakeState::STOPPED;

private:
    void DoIntake();
    void DoMoveDown();
    void IntakeScaled();
    void OuttakeScaled();
    void StopRunning();
    void DoStow();
    void DoAngleForBump();

    ctre::phoenix6::hardware::TalonFX m_spinner1{41, "team3045-2"};
    ctre::phoenix6::hardware::TalonFX m_spinner2{40, "team3045-2"};
    ctre::phoenix6::hardware::TalonFX m_tilter{24, "team3045-2"};
    ctre::phoenix6::hardware::CANcoder m_encoder{17, "team3045-2"};

    double m_intakeSpeed  = 40.0;
    double m_downPosition = 12.83;
    double m_minSpeed     = 80.0;

    CommandSwerveDrivetrain* m_drivetrain;
};

} // namespace subsystems
