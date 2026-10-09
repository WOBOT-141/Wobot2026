// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShooterV3 extends ParallelDeadlineGroup {
  private static double timeoutEnd_sec = 5;
  /** Creates a new AutonShooterV3. */
  public AutonShooterV3(
    Drivetrain drivetrain,
    Shooter shooter,
    Intake intake,
    MagazineAndFeed magazine) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().

    super(new InstantCommand().andThen(new WaitCommand(timeoutEnd_sec)));
    addCommands(new AutonShooterV2(drivetrain, shooter, intake, magazine));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
