// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MIdwaybutBetter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousCommands.StraightLineDrivePlus;
import frc.robot.commands.DriveCommands.StickCommand;
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
public class HALFswipeRIGHT extends SequentialCommandGroup {
  /** Creates a new HALFswipeRIGHT. */
  public HALFswipeRIGHT(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake, VisionServer visionServer, StickCommand stickCommand, AprilTagFilter aprilTagFilter, CommandXboxController controller) {
    StraightLineDrivePlus drive_over_ramp = new StraightLineDrivePlus(drivetrain, 0, 85, 0, wheels, gyro, 0.7);
    StraightLineDrivePlus drive_from_ramp_into_fuel = new StraightLineDrivePlus(drivetrain, 0, 55, 0, wheels, gyro, 1);
    StraightLineDrivePlus leftTurn_into_centerfuel = new StraightLineDrivePlus(drivetrain, -45, 45, -90, wheels, gyro, 0.4);
    StraightLineDrivePlus turnBack = new StraightLineDrivePlus(drivetrain, 135, 45, 90, wheels, gyro, 0.4);
    StraightLineDrivePlus drive_from_fuel_into_ramp = new StraightLineDrivePlus(drivetrain, 180, 55, 0, wheels, gyro, 1);
    StraightLineDrivePlus drive_over_ramp_back = new StraightLineDrivePlus(drivetrain, 180, 85, 0, wheels, gyro, 0.7);
    StraightLineDrivePlus drive_to_outpost = new StraightLineDrivePlus(drivetrain, 150, 140, 0, wheels, gyro, 1);
    StraightLineDrivePlus drive_to_tower = new StraightLineDrivePlus(drivetrain, -70, 145, 180, wheels, gyro, 1);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      drive_over_ramp
      .andThen(
        drive_from_ramp_into_fuel)
      .andThen(
        leftTurn_into_centerfuel)
      .andThen(
        turnBack)
      .andThen(
        drive_from_fuel_into_ramp)
      .andThen(
        drive_over_ramp_back)
      .andThen(
        drive_to_outpost)
      .andThen(
        drive_to_tower)
    );
  }
}
