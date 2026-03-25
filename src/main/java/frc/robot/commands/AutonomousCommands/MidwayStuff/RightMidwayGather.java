// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MidwayStuff;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtSpecificTag;
import frc.robot.commands.AutonomousCommands.HoodResetter;
import frc.robot.commands.AutonomousCommands.RotateByAngle;
import frc.robot.commands.AutonomousCommands.RotateByAnglePID;
import frc.robot.commands.AutonomousCommands.RotateToTag;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.commands.AutonomousCommands.setIntake;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.IntakeCommands.IntakeExtend;
import frc.robot.commands.ShooterCommands.AutonShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision.VisionServer;
import frc.robot.subsystems.Vision.AprilTags.AprilTagFilter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightMidwayGather extends SequentialCommandGroup {
  /** Creates a new MidwayGather. */    
  public RightMidwayGather(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake, VisionServer visionServer, StickCommand stickCommand, AprilTagFilter aprilTagFilter, CommandXboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    StraightLineDrive drive_over_ramp_to_center = new StraightLineDrive(drivetrain, 90, 180, wheels, gyro, true);
    StraightLineDrive drive_through_fuel = new StraightLineDrive(drivetrain, 0, 140, wheels, gyro, true);
    StraightLineDrive drive_back_to_line_up_with_ramp = new StraightLineDrive(drivetrain, 0, -118, wheels, gyro, false);
    StraightLineDrive drive_back_in_to_alliance_zone = new StraightLineDrive(drivetrain, -90, 183, wheels, gyro, false);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new HoodResetter(shooter, gyro)
            .andThen(
                drive_over_ramp_to_center)
                // .alongWith(new IntakeExtend(intake)).andThen(
                // new setIntake(intake, -0.6)).andThen(
                // new WaitCommand(0.5)))
            .andThen(
                drive_through_fuel)
                // .andThen(new setIntake(intake, 0.0)))
            .andThen(
                drive_back_to_line_up_with_ramp)
            .andThen(
                drive_back_in_to_alliance_zone)
            .andThen(
                // new RotateByAnglePID(20, gyro, drivetrain)
                // .andThen(new RotateToTag(11, visionServer, drivetrain)))
                new RotateByAngle(drivetrain, 20, gyro,false)
                .andThen(new AimAtSpecificTag(11, stickCommand, visionServer, shooter, gyro, aprilTagFilter, controller)))
            .andThen(
                new AutonShooter(shooter, magazine, 360, 65)));
            //     new RevShooter(280, 65, shooter)
            //       .alongWith(new WaitCommand(0.5)
            //       .andThen(new setFeed(magazine, 0.8))))
            // .andThen(
            //     new WaitCommand(5))
            // .andThen(
            //     new RevShooter(0, 0, shooter).alongWith(
            //     new setFeed(magazine, 0))));
  }
}
