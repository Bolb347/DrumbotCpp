// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>

using namespace ctre::phoenix6;

namespace subsystems { class CommandSwerveDrivetrain; }

namespace subsystems {
class Turret : public frc2::SubsystemBase {
 public:
  Turret(hardware::TalonFX* spinner, CommandSwerveDrivetrain *drivetrain);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void goToTargetFieldRelative(double target);
  void goToTarget(double target);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX *m_spinner;

  CommandSwerveDrivetrain* m_drivetrain;

  double m_target = 0;
  double m_ratio = 12;
};
}
