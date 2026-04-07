// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MIdwaybutBetter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class DoubleRIghtSwipe extends SequentialCommandGroup {
  /** Creates a new DoubleRIghtSwipe. */
  public DoubleRIghtSwipe(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake, VisionServer visionServer, StickCommand stickCommand, AprilTagFilter aprilTagFilter, CommandXboxController controller) {

    double bumpSpeed = 0.7;
    // FIRST CYCLE
    StraightLineDrivePlus drive_over_ramp = new StraightLineDrivePlus(drivetrain, 0, 85, 0, wheels, gyro, bumpSpeed);
    StraightLineDrivePlus drive_from_ramp_into_fuel = new StraightLineDrivePlus(drivetrain, 0, 45, 0, wheels, gyro, 1.45);
    StraightLineDrivePlus leftTurn_into_centerfuel = new StraightLineDrivePlus(drivetrain, -65, 70, -80, wheels, gyro, 0.65);
    StraightLineDrivePlus turnBack = new StraightLineDrivePlus(drivetrain, 115, 70, 80, wheels, gyro, 0.65);
    StraightLineDrivePlus drive_from_fuel_into_ramp = new StraightLineDrivePlus(drivetrain, 180, 45, -50, wheels, gyro, 1.4);
    StraightLineDrivePlus drive_over_ramp_back = new StraightLineDrivePlus(drivetrain, 180, 100,0, wheels, gyro, bumpSpeed);

    //SECOND CYCLE
    StraightLineDrivePlus drive_over_ramp_2 = new StraightLineDrivePlus(drivetrain, 0, 100, 0, wheels, gyro, bumpSpeed);
    StraightLineDrivePlus drive_from_ramp_into_fuel_2 = new StraightLineDrivePlus(drivetrain, 0, 45, 50, wheels, gyro, 1.45);
    StraightLineDrivePlus leftTurn_into_centerfuel_2 = new StraightLineDrivePlus(drivetrain, -45, 40, -90, wheels, gyro, 0.9);
    StraightLineDrivePlus gatherCenterFuel = new StraightLineDrivePlus(drivetrain, -90, 120, 0, wheels, gyro, 0.75);
    StraightLineDrivePlus line_back_to_ramp = new StraightLineDrivePlus(drivetrain, 110, 120, 90, wheels, gyro, 2.1);
    StraightLineDrivePlus drive_over_ramp_AGAIN = new StraightLineDrivePlus(drivetrain, 180, 120, -50, wheels, gyro, bumpSpeed);

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
        new WaitCommand(0.5))
      .andThen( //SECOND CYCLE
        drive_over_ramp_2)
      .andThen(
        drive_from_ramp_into_fuel_2)
      .andThen(
        leftTurn_into_centerfuel_2)
      .andThen(
        gatherCenterFuel)
      .andThen(
        line_back_to_ramp)
      .andThen(
        drive_over_ramp_AGAIN)
      
    );
  }
}
