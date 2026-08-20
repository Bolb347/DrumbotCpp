#include "robot/util/BallisticsSolver2.h"
#include "robot/Constants.h"
#include <numbers>
#include <cmath>
#include <units/math.h>            // for unit-aware math
#include <units/angle.h>           // for degree_t, radian_t
#include <units/length.h>          // for meter_t
#include <algorithm>               // for std::max

namespace util {

// Helper: degrees to radians
static inline double degToRad(double deg) { return deg * std::numbers::pi / 180.0; }
// Helper: radians to degrees
static inline double radToDeg(double rad) { return rad * 180.0 / std::numbers::pi; }

double BallisticsSolver2::DragSpeedRetention(double /*exitVel*/, double r) {
    return std::exp(-k * r);
}

double BallisticsSolver2::EffectiveRange(double r, double exitVel) {
    double retention    = DragSpeedRetention(exitVel, r);
    double avgRetention = (1.0 + retention) / 2.0;
    return r / avgRetention;
}

double BallisticsSolver2::CorrectTOF(double naiveTOF, double exitVel, double r) {
    double retention    = DragSpeedRetention(exitVel, r);
    double avgRetention = (1.0 + retention) / 2.0;
    return naiveTOF / avgRetention;
}

bool BallisticsSolver2::Scan(BallisticSolution& solution, double r, double deltaZ,
                              double threshold, ShotMode mode)
{
    double minReachableV = std::sqrt(g * (deltaZ + std::sqrt(r * r + deltaZ * deltaZ)));
    double vStart = std::max(minReachableV + 0.1, 1.0);
    double vMax   = constants::ShooterConstants::maxExitVel;

    if (vStart >= vMax) return false;

    constexpr double arcPreference = 0.23;
    double range   = vMax - vStart;
    double targetV = vStart + range * arcPreference;

    for (int i = depth; i >= 0; --i) {
        double currentV = vStart + (static_cast<double>(i) / depth) * (targetV - vStart);
        double rEff     = EffectiveRange(r, currentV);
        double v2       = currentV * currentV;
        double disc     = v2 * v2 - g * (g * rEff * rEff + 2.0 * deltaZ * v2);
        if (disc < 0) continue;

        double sqrtDisc  = std::sqrt(disc);
        double tanAngle  = (mode == ShotMode::HIGHARC)
                               ? (v2 + sqrtDisc) / (g * rEff)
                               : (v2 - sqrtDisc) / (g * rEff);
        // Convert atan result (radians) to degrees manually
        double fromVertical = 90.0 - radToDeg(std::atan(tanAngle));

        if (fromVertical >= constants::HoodConstants::minHood - threshold &&
            fromVertical <= constants::HoodConstants::maxHood + threshold)
        {
            solution.exitVelocity = currentV;
            solution.hoodAngle    = fromVertical;
            return true;
        }
    }

    // Clamped fallback
    double clampedFV = (mode == ShotMode::HIGHARC)
                           ? constants::HoodConstants::minHood
                           : constants::HoodConstants::maxHood;
    double launchRad = degToRad(90.0 - clampedFV);   // convert degrees to radians
    double cosA  = std::cos(launchRad);
    double tanA  = std::tan(launchRad);
    double rEff  = EffectiveRange(r, vMax);
    double denom = 2.0 * cosA * cosA * (rEff * tanA - deltaZ);

    if (denom > 0) {
        double requiredV = std::sqrt(g * rEff * rEff / denom);
        if (std::isfinite(requiredV) && requiredV <= constants::ShooterConstants::maxExitVel) {
            solution.exitVelocity = requiredV;
            solution.hoodAngle    = clampedFV;
            return true;
        }
    }
    return false;
}

BallisticsSolver2::BallisticSolution BallisticsSolver2::Solve(
    frc::Translation3d turretPos,
    frc::Translation3d robotVel,
    frc::Translation3d /*robotAccel*/,
    frc::Translation3d targetPos,
    double threshold,
    frc::Translation2d offset,
    ShotMode mode)
{
    BallisticSolution solution{};

    // Extract numeric values from unit types
    double omega = robotVel.Z().value();               // units::meter_t -> double
    double vRotX = omega * offset.Y().value();         // offset.Y() is meter_t
    double vRotY = -omega * offset.X().value();
    double totalVX = robotVel.X().value() + vRotX;
    double totalVY = robotVel.Y().value() + vRotY;

    double deltaZ  = (targetPos.Z() - turretPos.Z()).value();   // meter_t -> double
    double dist2d  = turretPos.ToTranslation2d().Distance(targetPos.ToTranslation2d()).value();

    double assumedTOF = dist2d / constants::ShooterConstants::maxExitVel * tofBSVal;

    // These dx, dy are distances (meters) as double
    double dx = totalVX * assumedTOF;
    double dy = totalVY * assumedTOF;

    // Build Translation2d using meter_t
    frc::Translation2d diff = targetPos.ToTranslation2d()
                                .operator-(turretPos.ToTranslation2d())
                                .operator-(frc::Translation2d{units::meter_t{dx}, units::meter_t{dy}});
    double r = std::max(diff.Norm().value(), 0.5);

    if (!Scan(solution, r, deltaZ, threshold, mode)) return solution;

    // First TOF refinement
    {
        double launchRad = degToRad(90.0 - solution.hoodAngle);
        double vExitH    = solution.exitVelocity * std::cos(launchRad);
        if (std::isfinite(vExitH) && vExitH > 0.5) {
            assumedTOF = CorrectTOF(r / vExitH, solution.exitVelocity, r) * tofBSVal;
            dx = totalVX * assumedTOF;
            dy = totalVY * assumedTOF;
            diff = targetPos.ToTranslation2d()
                       .operator-(turretPos.ToTranslation2d())
                       .operator-(frc::Translation2d{units::meter_t{dx}, units::meter_t{dy}});
            r = std::max(diff.Norm().value(), 0.5);
            if (!Scan(solution, r, deltaZ, threshold, mode)) return solution;
        }
    }

    // Iterative TOF refinement
    for (int iter = 0; iter < tofIterations; ++iter) {
        double launchRad = degToRad(90.0 - solution.hoodAngle);
        double vExitH    = solution.exitVelocity * std::cos(launchRad);
        if (!std::isfinite(vExitH) || vExitH < 0.5) break;

        double naiveTOF = r / vExitH;
        double newTOF   = CorrectTOF(naiveTOF, solution.exitVelocity, r);
        if (!std::isfinite(newTOF) || newTOF < 0.05 || newTOF > 5.0) break;

        bool converged = std::abs(newTOF - assumedTOF) < tofTolerance;
        assumedTOF = newTOF * tofBSVal;

        dx = totalVX * assumedTOF;
        dy = totalVY * assumedTOF;
        diff = targetPos.ToTranslation2d()
                   .operator-(turretPos.ToTranslation2d())
                   .operator-(frc::Translation2d{units::meter_t{dx}, units::meter_t{dy}});
        r = std::max(diff.Norm().value(), 0.5);

        if (!Scan(solution, r, deltaZ, threshold, mode)) break;
        if (converged) break;
    }

    // targetX, targetY are double distances
    double targetX = targetPos.X().value() - dx;
    double targetY = targetPos.Y().value() - dy;
    // atan2 returns radians, convert to degrees
    double turretAngle = radToDeg(std::atan2(targetY - turretPos.Y().value(),
                                             targetX - turretPos.X().value()));

    if (!std::isfinite(turretAngle)) return solution;

    solution.turretAngle  = turretAngle;
    solution.timeOfFlight = assumedTOF;
    solution.valid        = true;
    return solution;
}

} // namespace util