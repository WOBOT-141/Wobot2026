// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousCommands.HoodResetter;
import frc.robot.commands.AutonomousCommands.StraightLineDrivePlus;
import frc.robot.commands.IntakeCommands.IntakeExtend;
import frc.robot.commands.ShooterCommands.AutonShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SitShots extends SequentialCommandGroup {
  /** Creates a new SitShots. */
  public SitShots(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake) {
    StraightLineDrivePlus walkBack = new StraightLineDrivePlus(drivetrain, 180, 15, 0, wheels, gyro, 0.5);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodResetter(shooter, gyro)
      .andThen(
        walkBack
        .alongWith(new IntakeExtend(intake).withTimeout(2))) //Disallows string to get caught on hood;
      .andThen(
      new AutonShooter(shooter, magazine, 167, 67))
    );
  }
}
