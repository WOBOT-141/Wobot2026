// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousCommands.RotateByAngle;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.commands.ShooterCommands.RevShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftMidwayGather extends SequentialCommandGroup {
  /** Creates a new MidwayGather. */
  public LeftMidwayGather(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new StraightLineDrive(drivetrain, -90, 167, wheels, gyro).andThen(
      new StraightLineDrive(drivetrain, 0, 140, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, 0, -120, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, 90, 170, wheels, gyro)).andThen(
      new RotateByAngle(drivetrain, -20, gyro)).andThen(
      new RevShooter(260, 72, shooter)
    ));
  }
}
