// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.SwerveModule;

public class DepotLease {

  private DepotLease() {}

  public static Command create(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro) {

    return new StraightLineDrive(drivetrain, 90, 30, wheels, gyro, true)
        .andThen(new StraightLineDrive(drivetrain, 90, -30, wheels, gyro, true));
  }
}
