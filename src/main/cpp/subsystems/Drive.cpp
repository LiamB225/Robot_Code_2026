// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

Drive::Drive() = default;

// This method will be called once per scheduler run
void Drive::Periodic() {
    frc::Rotation2d angle{gyro.GetAngle()};
    m_poseEstimator.Update(angle, {
    frc::SwerveModulePosition{(units::meter_t)(flDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(frDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(blDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(brDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)}
    });
}

frc2::CommandPtr Drive::get_drive_command(
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
            drive = XRateLimiter.Calculate(frc::ApplyDeadband(get_drive_power(), 0.05)) * kRobotMaxSpeed;
            strafe = YRateLimiter.Calculate(frc::ApplyDeadband(get_strafe_power(), 0.05)) * kRobotMaxSpeed;
            rot = RotRateLimiter.Calculate(frc::ApplyDeadband(get_rot_power(), 0.05)) * kRobotRotMaxSpeed;
            
            SwerveDrive(drive, strafe, rot, false);
        }
    );
}

void Drive::SwerveDrive(
    units::meters_per_second_t xspeed,
    units::meters_per_second_t yspeed,
    units::radians_per_second_t rotspeed,
    bool fieldRelative
) {
    frc::ChassisSpeeds speeds{};
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed && fieldRelative) {
        speeds = {-xspeed, -yspeed, rotspeed};
    }
    else {
        speeds = {xspeed, yspeed, rotspeed};
    }
    frc::Rotation2d angle{};
    angle = m_poseEstimator.GetEstimatedPosition().Rotation();
    auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, angle) : speeds,
        0.02_s));
    m_kinematics.DesaturateWheelSpeeds(&states, kModuleMaxSpeed);
    auto [fl, fr, bl, br] = states;
    
    frc::Rotation2d flEncoderRotation{(units::radian_t)(flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};
    frc::Rotation2d frEncoderRotation{(units::radian_t)(frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};
    frc::Rotation2d blEncoderRotation{(units::radian_t)(blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};
    frc::Rotation2d brEncoderRotation{(units::radian_t)(brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};

    fl.Optimize(flEncoderRotation);
    fr.Optimize(frEncoderRotation);
    bl.Optimize(blEncoderRotation);
    br.Optimize(brEncoderRotation);

    fl.CosineScale(flEncoderRotation);
    fr.CosineScale(frEncoderRotation);
    bl.CosineScale(blEncoderRotation);
    br.CosineScale(brEncoderRotation);

    //Drive Control
    units::volt_t flDrivePIDVoltage = (units::volt_t)(flDrivePID.Calculate(
        flDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), fl.speed.value()));
    units::volt_t frDrivePIDVoltage = (units::volt_t)(frDrivePID.Calculate(
        frDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), fr.speed.value()));
    units::volt_t blDrivePIDVoltage = (units::volt_t)(blDrivePID.Calculate(
        blDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), bl.speed.value()));
    units::volt_t brDrivePIDVoltage = (units::volt_t)(brDrivePID.Calculate(
        brDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), br.speed.value()));

    units::volt_t flDriveFFVoltage = flDriveFF.Calculate(fl.speed);
    units::volt_t frDriveFFVoltage = frDriveFF.Calculate(fr.speed);
    units::volt_t blDriveFFVoltage = blDriveFF.Calculate(bl.speed);
    units::volt_t brDriveFFVoltage = brDriveFF.Calculate(br.speed);

    flDriveMotor.SetVoltage(flDrivePIDVoltage + flDriveFFVoltage);
    frDriveMotor.SetVoltage(frDrivePIDVoltage + frDriveFFVoltage);
    blDriveMotor.SetVoltage(blDrivePIDVoltage + blDriveFFVoltage);
    brDriveMotor.SetVoltage(brDrivePIDVoltage + brDriveFFVoltage);

    //Rotation Control
    units::volt_t flRotPIDVoltage = (units::volt_t)(flRotPID.Calculate(
        (units::radian_t)(flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), fl.angle.Radians()));
    units::volt_t frRotPIDVoltage = (units::volt_t)(frRotPID.Calculate(
        (units::radian_t)(frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), fr.angle.Radians()));
    units::volt_t blRotPIDVoltage = (units::volt_t)(blRotPID.Calculate(
        (units::radian_t)(blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), bl.angle.Radians()));
    units::volt_t brRotPIDVoltage = (units::volt_t)(brRotPID.Calculate(
        (units::radian_t)(brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), br.angle.Radians()));

    units::volt_t flRotFFVoltage = flRotFF.Calculate(flRotPID.GetSetpoint().velocity);
    units::volt_t frRotFFVoltage = frRotFF.Calculate(frRotPID.GetSetpoint().velocity);
    units::volt_t blRotFFVoltage = blRotFF.Calculate(blRotPID.GetSetpoint().velocity);
    units::volt_t brRotFFVoltage = brRotFF.Calculate(brRotPID.GetSetpoint().velocity);

    flRotMotor.SetVoltage(flRotPIDVoltage + flRotFFVoltage);
    frRotMotor.SetVoltage(frRotPIDVoltage + frRotFFVoltage);
    blRotMotor.SetVoltage(blRotPIDVoltage + blRotFFVoltage);
    brRotMotor.SetVoltage(brRotPIDVoltage + brRotFFVoltage);

    return;
}