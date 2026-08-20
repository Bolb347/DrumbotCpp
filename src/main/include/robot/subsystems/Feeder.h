#pragma once
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>

namespace subsystems {

class Feeder : public frc2::SubsystemBase {
public:
    Feeder(ctre::phoenix6::hardware::TalonFX* motor1,
           ctre::phoenix6::hardware::TalonFX* motor2);

    void GoToTargetSpeed(double targetSpeed);

private:
    ctre::phoenix6::hardware::TalonFX* m_motor1;
    ctre::phoenix6::hardware::TalonFX* m_motor2;
};

} // namespace subsystems
