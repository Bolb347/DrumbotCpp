#include "robot/generated/TunerConstants.h"
#include "robot/subsystems/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain() {
    return subsystems::CommandSwerveDrivetrain{
        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
    };
}