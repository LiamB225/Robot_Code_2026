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
constexpr int frontleftDriveMotorID = 1;
constexpr int frontleftRotMotorID = 2;
constexpr int frontleftRotEncoderID = 3;
constexpr int frontrightDriveMotorID = 4;
constexpr int frontrightRotMotorID = 5;
constexpr int frontrightRotEncoderID = 6;
constexpr int backleftDriveMotorID = 7;
constexpr int backleftRotMotorID = 8;
constexpr int backleftRotEncoderID = 9;
constexpr int backrightDriveMotorID = 10;
constexpr int backrightRotMotorID = 11;
constexpr int backrightRotEncoderID = 12;
constexpr int shooterMotorID = 13;
constexpr int upperMotorID = 14;
constexpr int lowerMotorID = 15;

}  // namespace OperatorConstants
