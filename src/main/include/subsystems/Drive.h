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
#include <cmath>
#include <math.h>

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

  frc2::CommandPtr get_drive_command(
    std::function<double(void)> get_drive_power,
    std::function<double(void)> get_strafe_power,
    std::function<double(void)> get_rot_power
  );
  
  void SwerveDrive(
    units::meters_per_second_t xspeed,
    units::meters_per_second_t yspeed,
    units::radians_per_second_t rotspeed,
    bool fieldRelative
  );

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //Kinematics Constants
  units::meters_per_second_t kModuleMaxSpeed = 1_mps;
  units::radians_per_second_t kRotMaxSpeed = 3.14_rad_per_s;
  units::radians_per_second_squared_t kRotMaxAccel = 100_rad_per_s_sq;

  //Robot Constants
  units::meters_per_second_t kRobotMaxSpeed = 1_mps;
  units::radians_per_second_t kRobotRotMaxSpeed = 3.14_rad_per_s;
  
  //Swerve Drive Kinematics
  units::meter_t kRobotLength = 0_m;
  units::meter_t kRobotWidth = 0_m;
  frc::SlewRateLimiter<units::scalar> XRateLimiter{5 / 1_s};
  frc::SlewRateLimiter<units::scalar> YRateLimiter{5 / 1_s};
  frc::SlewRateLimiter<units::scalar> RotRateLimiter{5 / 1_s};

  frc::SwerveDriveKinematics<4> m_kinematics{
    frc::Translation2d{kRobotLength / 2, kRobotWidth / 2},
    frc::Translation2d{kRobotLength / 2, -kRobotWidth / 2},
    frc::Translation2d{-kRobotLength / 2, kRobotWidth / 2},
    frc::Translation2d{-kRobotLength / 2, -kRobotWidth / 2}
  };

  //Gyro
  frc::ADIS16470_IMU gyro;

 

  //Front Left Module
  rev::spark::SparkMax frontleftDriveMotor{OperatorConstants::frontleftDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax frontleftRotMotor{OperatorConstants::frontleftRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder frontleftRotEncoder{OperatorConstants::frontleftRotEncoderID, "rio"};
  frc::PIDController flDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> flRotPID{0.0, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> flDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> flRotFF{0_V, 0_V / 1_rad_per_s};

  //Front Right Module
  rev::spark::SparkMax frontrightDriveMotor{OperatorConstants::frontrightDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax frontrightRotMotor{OperatorConstants::frontrightRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder frontrightRotEncoder{OperatorConstants::frontrightRotEncoderID, "rio"};
  frc::PIDController frDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> frRotPID{0.0, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> frDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> frRotFF{0_V, 0_V / 1_rad_per_s};

  //Back Left Module
  rev::spark::SparkMax backleftDriveMotor{OperatorConstants::backleftDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax backleftRotMotor{OperatorConstants::backleftRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder backleftRotEncoder{OperatorConstants::backleftRotEncoderID, "rio"};
  frc::PIDController blDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> blRotPID{0.0, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> blDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> blRotFF{0_V, 0_V / 1_rad_per_s};

  //Back Right Module
  rev::spark::SparkMax backrightDriveMotor{OperatorConstants::backrightDriveMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax backrightRotMotor{OperatorConstants::backrightRotMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder backrightRotEncoder{OperatorConstants::backrightRotEncoderID, "rio"};
  frc::PIDController brDrivePID{0.0, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> brRotPID{0.0, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> brDriveFF{0_V, 0_V / 1_mps, 0_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> brRotFF{0_V, 0_V / 1_rad_per_s};
 
  //Position Estimator
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator{m_kinematics, frc::Rotation2d{}, {
    frc::SwerveModulePosition{(units::meter_t)(frontleftDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(frontleftRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(frontrightDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(frontrightRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(backleftDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(backleftRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(backrightDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
      (units::radian_t)(backrightRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)}
  }, frc::Pose2d{}};
};
