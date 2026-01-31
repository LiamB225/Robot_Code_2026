// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants.h"

class Drive : public frc2::SubsystemBase {
 public:
  Drive();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr drivecommand(
    std::function<double(void)> drivepower,
    std::function<double(void)> strafepower,
    std::function<double(void)> rotpower
  );
  
  void SwerveDrive(
    units::meters_per_second_t xspeed,
    units::meters_per_second_t yspeed,
    units::radians_per_second_t rotspeed
  );

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Gyro
  frc::ADIS16470_IMU gyro;

  //Front Left Module
  rev::spark::SparkMax frontleftDriveMotor{OperatorConstants::frontleftDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax frontleftRotMotor{OperatorConstants::frontleftRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder frontleftRotEncoder{OperatorConstants::frontleftRotEncoderID, "rio"};
  frc::PIDController flDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> flRotPID{0.0, 0.0, 0.0, {0.0_rad_per_s, 0.0_rad_per_s_sq}};
  frc::SimpleMotorFeedforward<units::meters> flDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> flRotFF{0_V, 0_V / 1_rad_per_s};

  //Front Right Module
  rev::spark::SparkMax frontrightDriveMotor{OperatorConstants::frontrightDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax frontrightRotMotor{OperatorConstants::frontrightRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder frontrightRotEncoder{OperatorConstants::frontrightRotEncoderID, "rio"};
  frc::PIDController frDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> frRotPID{0.0, 0.0, 0.0, {0.0_rad_per_s, 0.0_rad_per_s_sq}};
  frc::SimpleMotorFeedforward<units::meters> frDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> frRotFF{0_V, 0_V / 1_rad_per_s};

  //Back Left Module
  rev::spark::SparkMax backleftDriveMotor{OperatorConstants::backleftDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax backleftRotMotor{OperatorConstants::backleftRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder backleftRotEncoder{OperatorConstants::backleftRotEncoderID, "rio"};
  frc::PIDController blDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> blRotPID{0.0, 0.0, 0.0, {0.0_rad_per_s, 0.0_rad_per_s_sq}};
  frc::SimpleMotorFeedforward<units::meters> blDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> blRotFF{0_V, 0_V / 1_rad_per_s};

  //Back Right Module
  rev::spark::SparkMax backrightDriveMotor{OperatorConstants::backrightDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax backrightRotMotor{OperatorConstants::backrightRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder backrightRotEncoder{OperatorConstants::backrightRotEncoderID, "rio"};
  frc::PIDController brDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> brRotPID{0.0, 0.0, 0.0, {0.0_rad_per_s, 0.0_rad_per_s_sq}};
  frc::SimpleMotorFeedforward<units::meters> brDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> brRotFF{0_V, 0_V / 1_rad_per_s};

};
