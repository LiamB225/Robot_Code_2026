// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Shooter.h"

Shooter::Shooter() = default;

// This method will be called once per scheduler run
void Shooter::Periodic() {}

frc2::CommandPtr Shooter::get_shoot_command() {
    return this->RunEnd(
        [this]() {
            shooterMotor.Set(.40);
            upperMotor.Set(.40);
            lowerMotor.Set(.40);
        },
        [this]() {
            shooterMotor.Set(0.0);
            upperMotor.Set(0.0);
            lowerMotor.Set(0.0);
        }
    );
}
