#pragma once
#include <frc2/command/CommandPtr.h>
#include "robot/subsystems/CommandSwerveDrivetrain.h"
#include "robot/subsystems/Feeder.h"
#include "robot/subsystems/Hopper.h"
#include "robot/subsystems/Intake.h"
#include "robot/subsystems/Shooter.h"

namespace commands {

class Autos {
public:
    Autos() = delete; // utility class

    static frc2::CommandPtr TurnToRelHeading(subsystems::CommandSwerveDrivetrain* drivetrain);

    static frc2::CommandPtr TestSubsystems(subsystems::CommandSwerveDrivetrain* drivetrain,
                                           subsystems::Shooter* s1,
                                           subsystems::Shooter* s2,
                                           subsystems::Feeder*  f1,
                                           subsystems::Feeder*  f2,
                                           subsystems::Hopper*  hopper,
                                           subsystems::Intake*  intake);

    static frc2::CommandPtr ToClimbHardCoded(subsystems::CommandSwerveDrivetrain* drivetrain);

    static bool AboutToCrossCenterLine(subsystems::CommandSwerveDrivetrain* drivetrain, double dangerZoneValue);
    static frc2::CommandPtr CenterLineResponse(subsystems::CommandSwerveDrivetrain* drivetrain, double dangerZoneValue);
};

} // namespace commands
