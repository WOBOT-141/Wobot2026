// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MoreBasicAutons;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousCommands.RotateByAngle;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.commands.ShooterCommands.RevShooter;
import frc.robot.commands.ShooterCommands.SendHoodToHome;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightMidwayGather extends SequentialCommandGroup {
  /** Creates a new MidwayGather. */
  public RightMidwayGather(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine) {
    // Add your commands in the addCommands() call, e.g.
    SendHoodToHome hood_to_home = new SendHoodToHome(shooter);
    StraightLineDrive drive_straight1 = new StraightLineDrive(drivetrain, 90, 187, wheels, gyro);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      hood_to_home.
      andThen(drive_straight1).andThen(
      new StraightLineDrive(drivetrain, 0, 140, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, 0, -120, wheels, gyro)).andThen(
      new StraightLineDrive(drivetrain, -90, 186, wheels, gyro)).andThen(
      new RotateByAngle(drivetrain, 25, gyro)).andThen(
      new RevShooter(260, 72, shooter).alongWith(
      new setFeed(magazine))
      
    ));
  }
}
