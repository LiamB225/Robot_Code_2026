// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  frc2::CommandPtr get_shoot_command();
  frc2::CommandPtr get_reverse_command();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::spark::SparkMax shooterMotor{OperatorConstants::shooterMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax upperMotor{OperatorConstants::upperMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax lowerMotor{OperatorConstants::lowerMotorID, rev::spark::SparkMax::MotorType::kBrushless};

  // frc::PIDController shooterPID{0.0, 0.0, 0.0};
  // frc::SimpleMotorFeedforward<units::radians> shooterFF{0.0_V, 0.0_V / 1_rad_per_s};
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
