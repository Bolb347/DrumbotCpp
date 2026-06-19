#include "robot/subsystems/Shooter.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include "robot/Constants.h"
#include "robot/util/Util.h"

#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <cmath>
#include <numbers>

using namespace subsystems;
using namespace ctre::phoenix6;

Shooter::Shooter(hardware::TalonFX* motor1, hardware::TalonFX* motor2,
                 hardware::TalonFX* motor3, hardware::TalonFX* motor4,
                 hardware::TalonFX* hood,
                 CommandSwerveDrivetrain* drivetrain)
    : m_motor1(motor1), m_motor2(motor2), m_motor3(motor3), m_motor4(motor4),
      m_hood(hood), m_drivetrain(drivetrain),
      m_vel1(motor1->GetRotorVelocity()),
      m_vel2(motor2->GetRotorVelocity()),
      m_vel3(motor3->GetRotorVelocity()),
      m_vel4(motor4->GetRotorVelocity())
{
    auto flyCfg  = constants::ShooterConstants::BuildFlywheelConfigC();
    auto flyCfgCC= constants::ShooterConstants::BuildFlywheelConfigCC();
    auto hoodCfg = constants::ShooterConstants::BuildHoodConfig();

    motor1->GetConfigurator().Apply(flyCfg);
    motor2->GetConfigurator().Apply(flyCfg);
    motor3->GetConfigurator().Apply(flyCfgCC);
    motor4->GetConfigurator().Apply(flyCfgCC);
    m_hood->GetConfigurator().Apply(hoodCfg);
    m_hood->SetPosition(0_tr);

    motor2->SetControl(controls::Follower{motor1->GetDeviceID(),
                       signals::MotorAlignmentValue::Aligned});
    motor3->SetControl(controls::Follower{motor1->GetDeviceID(),
                       signals::MotorAlignmentValue::Opposed});
    motor4->SetControl(controls::Follower{motor1->GetDeviceID(),
                       signals::MotorAlignmentValue::Opposed});
}

double Shooter::Rps() {
    BaseStatusSignal::RefreshAll(m_vel1, m_vel2, m_vel3, m_vel4);
    return (std::abs(m_vel1.GetValue().value()) +
            std::abs(m_vel2.GetValue().value()) +
            std::abs(m_vel3.GetValue().value()) +
            std::abs(m_vel4.GetValue().value())) / 4.0;
}

void Shooter::GoToTargetSpeed(double targetSpeed) {
    targetSpeed = util::Clamp(targetSpeed, -200.0, 200.0);
    controls::VelocityVoltage req{units::turns_per_second_t{targetSpeed}};
    m_motor1->SetControl(req);
    // motors 2-4 are followers, so only motor1 needs to be commanded
}

void Shooter::GoToTargetHoodAngle(double targetAngleDegFromVertical) {
    if (m_drivetrain->IsInHoodDeadzone()) {
        GoToTargetRot(0.0);
        return;
    }
    double targetRot = (targetAngleDegFromVertical - constants::HoodConstants::minHood)
                       / 360.0 * 37.0;
    m_hood->SetControl(controls::PositionVoltage{units::turn_t{targetRot}});
}

void Shooter::GoToTargetRot(double targetRot) {
    m_hood->SetControl(controls::PositionVoltage{units::turn_t{targetRot}});
}

void Shooter::SetExitVelTarget(double targetExitVelocity) {
    if (targetExitVelocity == 0.0) { GoToTargetSpeed(0.0); return; }

    constexpr double adjustment   = 0.75;
    constexpr double flywheelRadius = 0.0508; // 2 inches in meters
    double targetRPS = targetExitVelocity / (adjustment * 2.0 * std::numbers::pi * flywheelRadius);

    m_currTarget = targetRPS;
    GoToTargetSpeed(targetRPS);
    frc::SmartDashboard::PutNumber("DrumRpsDesired", targetRPS);
}

bool Shooter::IsAtTarget() {
    double tolerance = std::max(1.0, m_currTarget * 0.8);
    bool hasTarget = m_currTarget > 0.1;
    bool atSpeed   = hasTarget && std::abs(Rps() - m_currTarget) < tolerance;
    return m_stableDebouncer.Calculate(atSpeed);
}

void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("Drumspeed", Rps());
}
