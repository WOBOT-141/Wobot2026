// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MidwayStuff;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousCommands.RotateByAngle;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.commands.AutonomousCommands.setFeed;
import frc.robot.commands.AutonomousCommands.setIntake;
import frc.robot.commands.ShooterCommands.RevShooter;
import frc.robot.commands.ShooterCommands.SendHoodToHome;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightUturnMidwayGather extends SequentialCommandGroup {
  /** Creates a new MidwayGather. */
  public RightUturnMidwayGather(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    SendHoodToHome new HoodResetter(shooter, gyro) = new SendHoodToHome(shooter);
    StraightLineDrive drive_over_ramp_to_center = new StraightLineDrive(drivetrain, 0, 187, wheels, gyro, true);
    StraightLineDrive drive_through_fuel = new StraightLineDrive(drivetrain, -90, 140, wheels, gyro, true);
    StraightLineDrive drive_back_to_line_up_with_ramp = new StraightLineDrive(drivetrain, -90, -120, wheels, gyro, false);
    StraightLineDrive drive_back_in_to_alliance_zone = new StraightLineDrive(drivetrain, 0, -186, wheels, gyro, true);

    gyro.setGyroOffset(90);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new HoodResetter(shooter, gyro)
            .andThen(
                drive_over_ramp_to_center.andThen(
                new setIntake(intake, -0.6)).andThen(
                new WaitCommand(0.5)))
            .andThen(
                drive_through_fuel.andThen(
                new setIntake(intake, 0.0)))
            .andThen(
                drive_back_to_line_up_with_ramp)
            .andThen(
                drive_back_in_to_alliance_zone)
            .andThen(
                new RotateByAngle(drivetrain, 155, gyro, true))
            .andThen(
                new RevShooter(300, 65, shooter)
                  .alongWith(new WaitCommand(0.5)
                  .andThen(new setFeed(magazine,0.8))))
            );
  }
}
