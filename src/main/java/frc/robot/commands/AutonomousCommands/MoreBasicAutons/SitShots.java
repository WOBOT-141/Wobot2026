// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

public class SitShots {

  private SitShots() {}

  public static Command create(Shooter shooter, MagazineAndFeed magazine, IGyro gyro) {

  return shooter.homeHoodCommand()
      .andThen(shooter.autonShootCommand(magazine, 290, 72));

}
}