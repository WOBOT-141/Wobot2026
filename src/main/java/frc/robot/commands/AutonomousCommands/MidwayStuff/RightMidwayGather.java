// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MidwayStuff;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.AimAtSpecificTag;
import frc.robot.commands.AutonomousCommands.HoodResetter;
import frc.robot.commands.AutonomousCommands.RotateByAngle;
import frc.robot.commands.AutonomousCommands.RotateByAnglePID;
import frc.robot.commands.AutonomousCommands.RotateToTag;
import frc.robot.commands.AutonomousCommands.StraightLineDrive;
import frc.robot.commands.AutonomousCommands.StraightLineDrivePlus;
import frc.robot.commands.AutonomousCommands.setFeed;
import frc.robot.commands.AutonomousCommands.setIntake;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.IntakeCommands.IntakeExtend;
import frc.robot.commands.IntakeCommands.IntakeFuel;
import frc.robot.commands.ShooterCommands.AutonShooter;
import frc.robot.commands.ShooterCommands.RevShooter;
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
    StraightLineDrivePlus drive_over_ramp_to_center = new StraightLineDrivePlus(drivetrain, 90, 198, 0, wheels, gyro, 0.8);
    StraightLineDrivePlus drive_through_fuel = new StraightLineDrivePlus(drivetrain, 0, 140, 0, wheels, gyro, 0.8);
    StraightLineDrivePlus drive_back_to_line_up_with_ramp = new StraightLineDrivePlus(drivetrain, 180, 110, 0, wheels, gyro, 0.9);
    StraightLineDrivePlus drive_back_in_to_alliance_zone = new StraightLineDrivePlus(drivetrain, -90, 201, 20, wheels, gyro, 0.8);
    StraightLineDrivePlus drive_back_to_center = new StraightLineDrivePlus(drivetrain, 90, 180, 60, wheels, gyro, 0.9);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new HoodResetter(shooter, gyro)
            .andThen(
                drive_over_ramp_to_center
                .alongWith(new IntakeExtend(intake).withTimeout(3))
                .andThen(
                new setIntake(intake, 0.8)).andThen(
                new WaitCommand(0.5)))
            .andThen(
                drive_through_fuel
                .andThen(new setIntake(intake, 0.0)))
            .andThen(
                drive_back_to_line_up_with_ramp)
            .andThen(
                drive_back_in_to_alliance_zone)
            .andThen(
                new AutonShooter(shooter, magazine, 180, 65).withTimeout(4))
            .andThen(
                drive_back_to_center));
  }
}
