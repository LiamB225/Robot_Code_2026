// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() = default;

// This method will be called once per scheduler run
void Shooter::Periodic() {}

frc2::CommandPtr Shooter::get_shoot_command() {
    return frc2::cmd::Sequence(
        this->RunOnce( [this]() {
            shooterMotor.Set(0.4);
        }),
        frc2::cmd::Wait(0.5_s),
        this->RunOnce( [this]() {
            upperMotor.Set(0.4);
            lowerMotor.Set(0.164);
        })
    ).FinallyDo(
        [this]() {
            upperMotor.Set(0.0);
            lowerMotor.Set(0.0);
            shooterMotor.Set(0.0);
        }
    );

    // return this->StartEnd(
    //     [this]() {
    //         shooterMotor.Set(.40);
    //         upperMotor.Set(.40);
    //         lowerMotor.Set(.40);
    //     },
    //     [this]() {
    //         shooterMotor.Set(0.0);
    //         upperMotor.Set(0.0);
    //         lowerMotor.Set(0.0);
    //     }
    // );
}
