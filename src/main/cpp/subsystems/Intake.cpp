// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() = default;

// This method will be called once per scheduler run
void Intake::Periodic() {}

frc2::CommandPtr Intake::get_intake_command() {
    return this->StartEnd(
        [this]() {
            intakeMotor.Set(1.0);
        },
        [this]() {
            intakeMotor.Set(0.0);
        }
    );
}

frc2::CommandPtr Intake::get_outake_command() {
    return this->StartEnd(
        [this]() {
            intakeMotor.Set(-0.5);
        },
        [this]() {
            intakeMotor.Set(0.0);
        }
    );
}