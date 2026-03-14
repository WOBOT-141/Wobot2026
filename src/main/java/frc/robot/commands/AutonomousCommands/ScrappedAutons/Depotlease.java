// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.ScrappedAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousCommands.RotateByAngle;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.commands.AutonomousCommands.setIntake;
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
public class Depotlease extends SequentialCommandGroup {
  /** Creates a new Depotlease. */
  public Depotlease(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Intake intake, Shooter shooter, MagazineAndFeed magazine) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( // Hood resetter is needed later once hood is unfixed
        new StraightLineDrive(drivetrain, 90, 130, wheels, gyro, true).andThen(
            new StraightLineDrive(drivetrain, 0, 35, wheels, gyro, true))
            .alongWith(new IntakeExtend(intake).andThen(
                new setIntake(intake, -0.6))
            .andThen(
                new WaitCommand(0.5)))
            .andThen(
                new StraightLineDrive(drivetrain, 0, 45, wheels, gyro, true))
            .andThen(
                new StraightLineDrive(drivetrain, 0, 20, wheels, gyro, true))
            .andThen(
                new setIntake(intake, 0.0))
            .andThen(
                new StraightLineDrive(drivetrain, -75, 90, wheels, gyro, true))
            .andThen(
                new RotateByAngle(drivetrain, -45, gyro, false))
            .andThen(
                new AutonShooter(shooter, magazine, 310, 65))); //TODO tune
            // .andThen(
            //     new StraightLineDrive(drivetrain, -135, -47, wheels, gyro, true))
            // .andThen(
            //     new StraightLineDrive(drivetrain, -90, -12, wheels, gyro, true))); CLIMB PATH
  }
}
