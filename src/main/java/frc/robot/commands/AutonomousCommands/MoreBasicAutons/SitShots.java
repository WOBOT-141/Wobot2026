// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousCommands.HoodResetter;
import frc.robot.commands.IntakeCommands.IntakeFuel;
import frc.robot.commands.ShooterCommands.AutonShooter;
import frc.robot.commands.ShooterCommands.RevShooter;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SitShots extends SequentialCommandGroup {
  /** Creates a new SitShots. */
  public SitShots(Shooter shooter, MagazineAndFeed magazine, IGyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodResetter(shooter, gyro).andThen(
      new AutonShooter(shooter, magazine, 180, 70))
      // new RevShooter(300, 65, shooter))
      //   .alongWith(new WaitCommand(0.7)
      //   .andThen(new IntakeFuel(magazine,0.8))).andThen(
      //     new WaitCommand(5))
      //   .andThen(
      //     new RevShooter(0, 0, shooter).alongWith(
      //     new IntakeFuel(magazine, 0)))

    );
  }
}
