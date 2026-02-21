// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  m_drive.SetDefaultCommand(m_drive.get_drive_command(
    [this]() { return -m_driverController.GetLeftY(); },
    [this]() { return -m_driverController.GetLeftX(); },
    [this]() { return -m_driverController.GetRightX(); }
  ));

  m_driverController.Y().WhileTrue(m_shooter.get_shoot_command());
  m_driverController.RightTrigger().WhileTrue(m_intake.get_intake_command());
  m_driverController.LeftTrigger().WhileTrue(m_intake.get_outake_command());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous

  // PLACEHOLDER COMMAND FOR AUTO
  return m_drive.RunOnce([this](){});
}
