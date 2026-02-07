// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

Drive::Drive() = default;

// This method will be called once per scheduler run
void Drive::Periodic() {}

frc::CommandPtr Drive::get_drive_command(
    std::function<double(void)> get_drive_power,
    std::function<double(void)> get_strafe_power,
    std::function<double(void)> get_rot_power
) {
    return this->Run(
        [this, get_drive_power, get_strafe_power, get_rot_power]() {
            units::meters_per_second_t drive;
            units::meters_per_second_t strafe;
            units::radians_per_second_t rot;

            // Fill these variables with user input.
            drive = (units::meters_per_second_t)get_drive_power();
            strafe = (units::meters_per_second_t)get_strafe_power();
            rot = (units::meters_per_second_t)get_rot_power();
            
            SwerveDrive(drive, strafe, rot);
        }
    );
}

void Drive::SwerveDrive(
    units::meters_per_second_t xspeed,
    units::meters_per_second_t yspeed,
    units::radians_per_second_t rotspeed
) {
    return;
}