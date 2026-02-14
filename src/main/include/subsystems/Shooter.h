// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include "Constants.h"

class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  frc2::CommandPtr get_shoot_command();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  rev::spark::SparkMax shooterMotor{OperatorConstants::shooterMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax upperMotor{OperatorConstants::upperMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax lowerMotor{OperatorConstants::lowerMotorID, rev::spark::SparkMax::MotorType::kBrushless};
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
