#pragma once

#include "ctre/phoenix6/configs/Configuration.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/current.h>
#include <units/voltage.h>
#include <units/time.h>

using namespace ctre::phoenix6;

namespace constants {

inline const CANBus kMainCanbus{"team3045"};
inline const CANBus kSecondaryCanbus{"team3045-2"};

inline constexpr double kBlueTarget = 45.0;
inline constexpr double kRedTarget  = 135.0;

struct OperatorConstants {
    static constexpr int kDriverControllerPort = 0;
};

// ─────────────────── Hood ───────────────────
struct HoodConstants {
    static constexpr double minHood    = 0.5;
    static constexpr double maxHood    = 30.0;
    static constexpr double frontOfHub = 60.0;
};

// ─────────────────── Feeder ───────────────────
struct FeederConstants {
    static constexpr double feederTargetSpeed = 80.0;
    static constexpr double kP = 0.3,  kI = 0.0, kD = 0.0;
    static constexpr double kG = 0.0,  kS = 0.0, kA = 0.0, kV = 12.0;
    
    static constexpr double supplyLimit = 30.0;
    static constexpr double statorLimit = 40.0;

    static configs::TalonFXConfiguration BuildConfig() {
        return configs::TalonFXConfiguration{}
            .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(units::ampere_t{supplyLimit})
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(units::ampere_t{statorLimit})
                .WithStatorCurrentLimitEnable(true))
            .WithSlot0(configs::Slot0Configs{}
                .WithKP(kP).WithKI(kI).WithKD(kD)
                .WithKG(kG).WithKS(kS).WithKA(kA).WithKV(kV));
    }
};

// ─────────────────── Hopper ───────────────────
struct HopperConstants {
    static constexpr double kP = 0.2,  kI = 0.0, kD = 0.0;
    static constexpr double kG = 0.0,  kS = 0.4, kA = 0.0, kV = 0.115;
    
    static constexpr double supplyLimit = 25.0;
    static constexpr double statorLimit = 40.0;

    static configs::TalonFXConfiguration BuildConfig() {
        return configs::TalonFXConfiguration{}
            .WithMotorOutput(configs::MotorOutputConfigs{}
                .WithNeutralMode(signals::NeutralModeValue::Coast)
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive))
            .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(units::ampere_t{supplyLimit})
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(units::ampere_t{statorLimit})
                .WithStatorCurrentLimitEnable(true))
            .WithSlot0(configs::Slot0Configs{}
                .WithKP(kP).WithKI(kI).WithKD(kD)
                .WithKG(kG).WithKS(kS).WithKA(kA).WithKV(kV));
    }
};

// ─────────────────── Intake ───────────────────
struct IntakeConstants {
    static constexpr double kP = 0.35, kI = 0.0, kD = 0.0;
    static constexpr double kG = 0.0,  kS = 0.18, kA = 0.0, kV = 0.115;
    
    static constexpr double spinnerSupplyLimit = 35.0;
    static constexpr double spinnerStatorLimit = 60.0;
    
    static constexpr double kP1 = 2.5, kP2 = 1.5;
    static constexpr double kI1 = 0.0, kD1 = 0.0, kG1 = 0.0, kS1 = 0.0, kA1 = 0.0, kV1 = 0.0;
    
    static constexpr double tilterSupplyLimit = 30.0;
    static constexpr double tilterStatorLimit = 50.0;

    static configs::TalonFXConfiguration BuildSpinnerConfigC() {
        return configs::TalonFXConfiguration{}
            .WithMotorOutput(configs::MotorOutputConfigs{}
                .WithNeutralMode(signals::NeutralModeValue::Coast)
                .WithInverted(signals::InvertedValue::Clockwise_Positive))
            .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(units::ampere_t{spinnerSupplyLimit})
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(units::ampere_t{spinnerStatorLimit})
                .WithStatorCurrentLimitEnable(true))
            .WithSlot0(configs::Slot0Configs{}
                .WithKP(kP).WithKI(kI).WithKD(kD)
                .WithKG(kG).WithKS(kS).WithKA(kA).WithKV(kV));
    }

    static configs::TalonFXConfiguration BuildSpinnerConfigCC() {
        return BuildSpinnerConfigC()
            .WithMotorOutput(configs::MotorOutputConfigs{}
                .WithNeutralMode(signals::NeutralModeValue::Coast)
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive));
    }

    static configs::TalonFXConfiguration BuildTilterConfig() {
        return configs::TalonFXConfiguration{}
            .WithMotorOutput(configs::MotorOutputConfigs{}
                .WithNeutralMode(signals::NeutralModeValue::Brake)
                .WithInverted(signals::InvertedValue::Clockwise_Positive))
            .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(units::ampere_t{tilterSupplyLimit})
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(units::ampere_t{tilterStatorLimit})
                .WithStatorCurrentLimitEnable(true))
            .WithVoltage(configs::VoltageConfigs{}
                .WithPeakForwardVoltage(12.0_V)
                .WithPeakReverseVoltage(-12.0_V))
            .WithSlot0(configs::Slot0Configs{}
                .WithKP(kP1).WithKI(kI1).WithKD(kD1)
                .WithKG(kG1).WithKS(kS1).WithKA(kA1).WithKV(kV1))
            .WithSlot1(configs::Slot1Configs{}
                .WithKP(kP2).WithKI(kI1).WithKD(kD1)
                .WithKG(kG1).WithKS(kS1).WithKA(kA1).WithKV(kV1));
    }
};

// ─────────────────── Shooter ───────────────────
struct ShooterConstants {
    static constexpr double maxExitVel = 12.0;

    static constexpr double kP = 0.47, kI = 0.0, kD = 0.0;
    static constexpr double kG = 0.0,  kS = 0.44, kA = 0.14, kV = 0.124;
    
    static constexpr double flywheelSupplyLimit = 40.0;
    static constexpr double flywheelStatorLimit = 80.0;
    static constexpr double flywheelRampPeriod  = 0.25;

    static constexpr double kP1 = 2.0, kI1 = 0.0, kD1 = 0.0;
    static constexpr double kG1 = 0.0, kS1 = 0.0,  kA1 = 0.0, kV1 = 0.0;
    
    static constexpr double hoodSupplyLimit = 20.0;
    static constexpr double hoodStatorLimit = 40.0;

    static configs::TalonFXConfiguration BuildFlywheelConfigC() {
        return configs::TalonFXConfiguration{}
            .WithMotorOutput(configs::MotorOutputConfigs{}
                .WithNeutralMode(signals::NeutralModeValue::Coast)
                .WithInverted(signals::InvertedValue::Clockwise_Positive))
            .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(units::ampere_t{flywheelSupplyLimit})
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(units::ampere_t{flywheelStatorLimit})
                .WithStatorCurrentLimitEnable(true))
            .WithOpenLoopRamps(configs::OpenLoopRampsConfigs{}
                .WithDutyCycleOpenLoopRampPeriod(units::second_t{flywheelRampPeriod})
                .WithVoltageOpenLoopRampPeriod(units::second_t{flywheelRampPeriod}))
            .WithSlot0(configs::Slot0Configs{}
                .WithKP(kP).WithKI(kI).WithKD(kD)
                .WithKG(kG).WithKS(kS).WithKA(kA).WithKV(kV));
    }

    static configs::TalonFXConfiguration BuildFlywheelConfigCC() {
        return BuildFlywheelConfigC()
            .WithMotorOutput(configs::MotorOutputConfigs{}
                .WithNeutralMode(signals::NeutralModeValue::Coast)
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive));
    }

    static configs::TalonFXConfiguration BuildHoodConfig() {
        return configs::TalonFXConfiguration{}
            .WithMotorOutput(configs::MotorOutputConfigs{}
                .WithNeutralMode(signals::NeutralModeValue::Brake)
                .WithInverted(signals::InvertedValue::CounterClockwise_Positive))
            .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                .WithSupplyCurrentLimit(units::ampere_t{hoodSupplyLimit})
                .WithSupplyCurrentLimitEnable(true)
                .WithStatorCurrentLimit(units::ampere_t{hoodStatorLimit})
                .WithStatorCurrentLimitEnable(true))
            .WithVoltage(configs::VoltageConfigs{}
                .WithPeakForwardVoltage(12.0_V)
                .WithPeakReverseVoltage(-12.0_V))
            .WithSlot0(configs::Slot0Configs{}
                .WithKP(kP1).WithKI(kI1).WithKD(kD1)
                .WithKG(kG1).WithKS(kS1).WithKA(kA1).WithKV(kV1));
    }
};

} // namespace constants