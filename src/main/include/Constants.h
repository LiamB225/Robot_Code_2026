// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
constexpr int flDriveMotorID = 1;
constexpr int flRotMotorID = 2;
constexpr int flRotEncoderID = 3;
constexpr int frDriveMotorID = 4;
constexpr int frRotMotorID = 5;
constexpr int frRotEncoderID = 6;
constexpr int blDriveMotorID = 7;
constexpr int blRotMotorID = 8;
constexpr int blRotEncoderID = 9;
constexpr int brDriveMotorID = 10;
constexpr int brRotMotorID = 11;
constexpr int brRotEncoderID = 12;
constexpr int shooterMotorID = 13;
constexpr int upperMotorID = 14;
constexpr int lowerMotorID = 15;
constexpr int intakeMotorID = 16;
}  // namespace OperatorConstants
