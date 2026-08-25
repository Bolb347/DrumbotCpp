#include "robot/subsystems/Intake.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include "robot/Constants.h"

#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <cmath>

using namespace subsystems;
using namespace ctre::phoenix6;

Intake::Intake(CommandSwerveDrivetrain* drivetrain) : m_drivetrain(drivetrain) {
    m_spinner1.GetConfigurator().Apply(constants::IntakeConstants::BuildSpinnerConfigC());
    m_spinner2.GetConfigurator().Apply(constants::IntakeConstants::BuildSpinnerConfigCC());
    m_tilter.GetConfigurator().Apply(constants::IntakeConstants::BuildTilterConfig());
    m_tilter.SetPosition(0_tr);
}

void Intake::DoIntake() {
    controls::VelocityVoltage vel{units::turns_per_second_t{m_intakeSpeed}};
    vel.EnableFOC = true;
    m_spinner1.SetControl(vel);
    m_spinner2.SetControl(vel);
}

void Intake::DoMoveDown() {
    m_tilter.SetControl(controls::PositionVoltage{units::turn_t{m_downPosition}}.WithEnableFOC(true));
}

void Intake::IntakeScaled()  { state = IntakeState::INTAKING; }
void Intake::OuttakeScaled() { state = IntakeState::OUTTAKING; }
void Intake::StopRunning()   { state = IntakeState::STOPPED; }

void Intake::DoStow() {
    m_tilter.SetControl(controls::PositionVoltage{0_tr}.WithSlot(0).WithEnableFOC(true));
}

void Intake::DoAngleForBump() {
    m_tilter.SetControl(controls::PositionVoltage{units::turn_t{14}}.WithSlot(0).WithEnableFOC(true));
}

frc2::CommandPtr Intake::Run()     { return RunOnce([this] { IntakeScaled(); }); }
frc2::CommandPtr Intake::Outtake() { return RunOnce([this] { OuttakeScaled(); }); }
frc2::CommandPtr Intake::Stop()    { return RunOnce([this] { StopRunning(); }); }

frc2::CommandPtr Intake::ToggleIntake() {
    return RunOnce([this] {
        if (state == IntakeState::INTAKING || state == IntakeState::OUTTAKING)
            state = IntakeState::STOPPED;
        else
            state = IntakeState::INTAKING;
    });
}

frc2::CommandPtr Intake::MoveDown()    { return RunOnce([this] { DoMoveDown(); }); }
frc2::CommandPtr Intake::Stow()        { return RunOnce([this] { DoStow(); }); }
frc2::CommandPtr Intake::AngleForBump(){ return RunOnce([this] { DoAngleForBump(); }); }

frc2::CommandPtr Intake::SlowPush() {
    return RunOnce([this] {
        DoIntake();
        m_tilter.SetControl(controls::PositionVoltage{units::turn_t{1.9}}.WithSlot(1).WithEnableFOC(true));
    });
}

frc2::CommandPtr Intake::AutoZero() {
    auto timer = std::make_shared<frc::Timer>();

    return this->RunEnd(
        [this, timer] {
            StopRunning();
            m_tilter.SetControl(controls::DutyCycleOut{0.5});
        },
        [this] {
            m_tilter.SetControl(controls::DutyCycleOut{0.0});
            m_tilter.SetPosition(units::turn_t{12.83});
        }
    )
    .BeforeStarting([timer] {
        timer->Reset();
        timer->Start();
    })
    .Until([this, timer] {
        bool inrushPassed = timer->HasElapsed(0.2_s);
        bool currentExceeded = m_tilter.GetStatorCurrent().GetValue().value() > 40.0;
        
        return inrushPassed && currentExceeded;
    });
}

void Intake::Periodic() {
    double currentSpeed = 0.0;
    if (state == IntakeState::INTAKING || state == IntakeState::OUTTAKING) {
        auto speeds = m_drivetrain->GetState().Speeds;
        currentSpeed = std::sqrt(speeds.vx.value() * speeds.vx.value() +
                                 speeds.vy.value() * speeds.vy.value());
    }

    if (state == IntakeState::INTAKING) {
        double velRPS = 30.0 * currentSpeed + m_minSpeed;
        controls::VelocityVoltage vel{units::turns_per_second_t{velRPS}};
        vel.EnableFOC = true;
        frc::SmartDashboard::PutNumber("IntakeTargetSpeed", velRPS);
        m_spinner1.SetControl(vel);
        m_spinner2.SetControl(vel);
    } else if (state == IntakeState::OUTTAKING) {
        controls::VelocityVoltage vel{units::turns_per_second_t{-80.0}};
        vel.EnableFOC = true;
        frc::SmartDashboard::PutNumber("IntakeTargetSpeed", -80.0);
        m_spinner1.SetControl(vel);
        m_spinner2.SetControl(vel);
    } else {
        controls::VelocityVoltage vel{units::turns_per_second_t{0.0}};
        vel.EnableFOC = true;
        m_spinner1.SetControl(vel);
        m_spinner2.SetControl(vel);
    }

    double avgSpeed = (m_spinner1.GetRotorVelocity().GetValue().value() +
                       m_spinner2.GetRotorVelocity().GetValue().value()) / 2.0;
    frc::SmartDashboard::PutNumber("IntakeSpeed", avgSpeed);
}
