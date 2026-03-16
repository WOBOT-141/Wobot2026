// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousCommands.HoodResetter;
import frc.robot.commands.ShooterCommands.AutonShooter;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SitShots extends SequentialCommandGroup {
  
   private SitShots() {}

  public static Command create(Shooter shooter, MagazineAndFeed magazine, IGyro gyro) {

    return new HoodResetter(shooter, gyro)
        .andThen(new AutonShooter(shooter, magazine, 290, 72));

  }
}
