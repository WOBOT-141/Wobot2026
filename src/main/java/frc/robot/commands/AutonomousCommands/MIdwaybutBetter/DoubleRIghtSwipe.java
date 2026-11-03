// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MIdwaybutBetter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousCommands.HoodResetter;
import frc.robot.commands.AutonomousCommands.RotateToTag;
import frc.robot.commands.AutonomousCommands.StraightLineDrivePlus;
import frc.robot.commands.AutonomousCommands.setIntake;
import frc.robot.commands.DriveCommands.StickCommand;
import frc.robot.commands.IntakeCommands.IntakeExtend;
import frc.robot.commands.IntakeCommands.IntakeRetract;
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
public class DoubleRIghtSwipe extends SequentialCommandGroup {
  /** Creates a new DoubleRIghtSwipe. */
  public DoubleRIghtSwipe(Drivetrain drivetrain, SwerveModule[] wheels, IGyro gyro, Shooter shooter, MagazineAndFeed magazine, Intake intake, VisionServer visionServer, StickCommand stickCommand, AprilTagFilter aprilTagFilter, CommandXboxController controller) {

    double bumpSpeed = 0.8;
    double intakeSpeed = 1;
    // FIRST CYCLE
    StraightLineDrivePlus drive_over_ramp = new StraightLineDrivePlus(drivetrain, 0, 85, 0, wheels, gyro, bumpSpeed);
    StraightLineDrivePlus drive_from_ramp_into_fuel = new StraightLineDrivePlus(drivetrain, 0, 75, 0, wheels, gyro, 0.99);
    StraightLineDrivePlus leftTurn_into_centerfuel = new StraightLineDrivePlus(drivetrain, -65, 50, -80, wheels, gyro, 0.65);
    StraightLineDrivePlus turnBack = new StraightLineDrivePlus(drivetrain, 115, 50, 90, wheels, gyro, 0.65);
    StraightLineDrivePlus drive_from_fuel_into_ramp = new StraightLineDrivePlus(drivetrain, 180, 45, -0, wheels, gyro, 0.99);
    StraightLineDrivePlus drive_straight_to_ramp = new StraightLineDrivePlus(drivetrain, 150, 85, 40, wheels, gyro, 0.99);
    StraightLineDrivePlus drive_over_ramp_back = new StraightLineDrivePlus(drivetrain, 180, 120,-15, wheels, gyro, bumpSpeed);

    //SECOND CYCLE
    StraightLineDrivePlus drive_over_ramp_2 = new StraightLineDrivePlus(drivetrain, 0, 100, 0, wheels, gyro, bumpSpeed);
    StraightLineDrivePlus drive_from_ramp_into_fuel_2 = new StraightLineDrivePlus(drivetrain, 0, 50, 45, wheels, gyro, 0.99);
    StraightLineDrivePlus leftTurn_into_centerfuel_2 = new StraightLineDrivePlus(drivetrain, -45, 40, -90, wheels, gyro, 0.9);
    StraightLineDrivePlus gatherCenterFuel = new StraightLineDrivePlus(drivetrain, -90, 110, -20, wheels, gyro, 0.75);
    StraightLineDrivePlus line_back_to_ramp = new StraightLineDrivePlus(drivetrain, 110, 125, 90, wheels, gyro, 0.99);
    StraightLineDrivePlus drive_over_ramp_AGAIN = new StraightLineDrivePlus(drivetrain, 180, 120, -45, wheels, gyro, bumpSpeed);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoodResetter(shooter, gyro)
      .alongWith(
        drive_over_ramp
        .andThen(new setIntake(intake, intakeSpeed)))
      .andThen(
        drive_from_ramp_into_fuel
        .alongWith(new IntakeExtend(intake).withTimeout(1)))
      .andThen(
        leftTurn_into_centerfuel)
        // Longer Route
      // .andThen(
      //   turnBack)
      // .andThen(
      //   drive_from_fuel_into_ramp
      //   .andThen(new setIntake(intake, 0)))
        
      .andThen(
        drive_straight_to_ramp
        .andThen(new setIntake(intake, 0)))
      .andThen(
        drive_over_ramp_back)
        .andThen(new RotateToTag(11, visionServer, drivetrain).withTimeout(0.01)
        .alongWith(
        new AutonShooter(shooter, magazine, 180, 65).withTimeout(2)
        .alongWith(new IntakeRetract(intake).withTimeout(0.5))
        // .andThen(new setIntake(intake, -0.4))
        .andThen(new WaitCommand(0.5))))
      .andThen( //SECOND CYCLE
        drive_over_ramp_2
        // .andThen(new setIntake(intake, 0))
        .alongWith(new IntakeExtend(intake).withTimeout(1)))
      .andThen(
        drive_from_ramp_into_fuel_2
        .alongWith(new setIntake(intake, intakeSpeed)))
      .andThen(
        leftTurn_into_centerfuel_2
        .andThen(new setIntake(intake, intakeSpeed)))
      .andThen(
        gatherCenterFuel)
      .andThen(
        line_back_to_ramp
        .andThen(new setIntake(intake, 0)))
      .andThen(
        drive_over_ramp_AGAIN)
        .andThen(new RotateToTag(11, visionServer, drivetrain).withTimeout(0.01)
          .alongWith(
            new AutonShooter(shooter, magazine, 160, 70)
            .alongWith(new IntakeRetract(intake).withTimeout(0.5))
            .withTimeout(3))
            .andThen(new IntakeExtend(intake)))
      
    );
  }
}
