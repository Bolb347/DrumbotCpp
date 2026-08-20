#include "robot/subsystems/Feeder.h"
#include "robot/Constants.h"
#include "robot/util/Util.h"
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <units/angular_velocity.h>

namespace subsystems {

Feeder::Feeder(ctre::phoenix6::hardware::TalonFX* motor1,
               ctre::phoenix6::hardware::TalonFX* motor2)
    : m_motor1(motor1), m_motor2(motor2)
{
    auto cfg = constants::FeederConstants::BuildConfig();
    m_motor1->GetConfigurator().Apply(cfg);
    m_motor2->GetConfigurator().Apply(cfg);
}

void Feeder::GoToTargetSpeed(double targetSpeed) {
    targetSpeed = util::Clamp(targetSpeed,
                               -constants::FeederConstants::feederTargetSpeed,
                               120.0);
    ctre::phoenix6::controls::VelocityVoltage req1{units::turns_per_second_t{ targetSpeed}};
    ctre::phoenix6::controls::VelocityVoltage req2{units::turns_per_second_t{-targetSpeed}};
    req1.EnableFOC = true;
    req2.EnableFOC = true;
    m_motor1->SetControl(req1);
    m_motor2->SetControl(req2);
}

} // namespace subsystems
