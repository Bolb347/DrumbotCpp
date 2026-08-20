#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>

namespace util {

class BallisticsSolver2 {
public:
    struct BallisticSolution {
        BallisticSolution() : valid(false), hoodAngle(0), turretAngle(0), exitVelocity(0), timeOfFlight(0) {}
        bool valid;
        double hoodAngle;
        double turretAngle;
        double exitVelocity;
        double timeOfFlight;
    };

    enum class ShotMode { HIGHARC, LOWARC };

    BallisticSolution Solve(frc::Translation3d turretPos,
                            frc::Translation3d robotVel,
                            frc::Translation3d robotAccel,
                            frc::Translation3d targetPos,
                            double threshold,
                            frc::Translation2d offset,
                            ShotMode mode);

private:
    static constexpr double g = 9.81;
    static constexpr int depth = 16;
    static constexpr int tofIterations = 3;
    static constexpr double tofTolerance = 0.01;

    // Drag constants
    static constexpr double Cd = 0.47;
    static constexpr double rho = 1.225;
    static constexpr double ballRadius = 0.0508;
    static constexpr double ballMass = 0.223;
    static constexpr double area = 3.14159265358979 * ballRadius * ballRadius;
    static constexpr double k = (0.5 * Cd * rho * area) / ballMass;

    const double tofBSVal = 0.8;

    double DragSpeedRetention(double exitVel, double r);
    double EffectiveRange(double r, double exitVel);
    double CorrectTOF(double naiveTOF, double exitVel, double r);
    bool Scan(BallisticSolution& solution, double r, double deltaZ, double threshold, ShotMode mode);
};

} // namespace util
