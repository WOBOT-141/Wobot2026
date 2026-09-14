// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MIdwaybutBetter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAtSpecificTag;
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
public class RRRightMidwayAuton extends SequentialCommandGroup {
  /** Creates a new RRRightMidwayAuton. */
  public RRRightMidwayAuton(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake, VisionServer visionServer, StickCommand stickCommand, AprilTagFilter aprilTagFilter, CommandXboxController controller) {

    StraightLineDrivePlus drive_over_ramp = new StraightLineDrivePlus(drivetrain, 0, 85, 0, wheels, gyro, 0.7);
    StraightLineDrivePlus drive_from_ramp_into_fuel = new StraightLineDrivePlus(drivetrain, 0, 55, 0, wheels, gyro, 1);
    StraightLineDrivePlus leftTurn_into_centerfuel = new StraightLineDrivePlus(drivetrain, -45, 45, -90, wheels, gyro, 0.4);
    StraightLineDrivePlus gatherCenterFuel = new StraightLineDrivePlus(drivetrain, -90, 135, 0, wheels, gyro, 0.6);
    StraightLineDrivePlus turnAround = new StraightLineDrivePlus(drivetrain, 135, 90, 180, wheels, gyro, 0.6);
    StraightLineDrivePlus lineUpToRamp = new StraightLineDrivePlus(drivetrain, 100, 105, 240, wheels, gyro, 0.6);
    StraightLineDrivePlus drive_over_ramp_AGAIN = new StraightLineDrivePlus(drivetrain, 180, 100, 0, wheels, gyro, 0.7);


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                drive_over_ramp
                .andThen(
                drive_from_ramp_into_fuel)
                .andThen(
                  leftTurn_into_centerfuel)
                .andThen(
                  gatherCenterFuel)
                .andThen(
                  turnAround)
                .andThen(
                  lineUpToRamp)
                .andThen(
                  drive_over_ramp_AGAIN)
    );
  }
}
