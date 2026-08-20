#include "robot/subsystems/Hopper.h"
#include "robot/Constants.h"
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <units/angular_velocity.h>

namespace subsystems {

Hopper::Hopper() {
    m_motor.GetConfigurator().Apply(constants::HopperConstants::BuildConfig());
}

void Hopper::Feed() {
    ctre::phoenix6::controls::VelocityVoltage req{units::turns_per_second_t{m_feedSpeed}};
    req.EnableFOC = true;
    m_motor.SetControl(req);
}

void Hopper::DoOuttake() {
    ctre::phoenix6::controls::VelocityVoltage req{units::turns_per_second_t{-m_feedSpeed}};
    req.EnableFOC = true;
    m_motor.SetControl(req);
}

void Hopper::DoStop() {
    ctre::phoenix6::controls::VelocityVoltage req{units::turns_per_second_t{0}};
    req.EnableFOC = true;
    m_motor.SetControl(req);
}

frc2::CommandPtr Hopper::Run() {
    return this->RunOnce([this] { Feed(); });
}

frc2::CommandPtr Hopper::Outtake() {
    return this->RunOnce([this] { DoOuttake(); });
}

frc2::CommandPtr Hopper::Stop() {
    return this->RunOnce([this] { DoStop(); });
}

} // namespace subsystems
