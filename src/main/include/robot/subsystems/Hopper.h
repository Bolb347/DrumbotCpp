#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace subsystems {

class Hopper : public frc2::SubsystemBase {
public:
    Hopper();

    frc2::CommandPtr Run();
    frc2::CommandPtr Outtake();
    frc2::CommandPtr Stop();

private:
    void Feed();
    void DoOuttake();
    void DoStop();

    ctre::phoenix6::hardware::TalonFX m_motor{6, "team3045-2"};
    double m_feedSpeed = 100.0;
};

} // namespace subsystems
