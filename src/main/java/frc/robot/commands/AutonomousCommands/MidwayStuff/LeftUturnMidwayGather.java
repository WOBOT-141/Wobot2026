// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.MidwayStuff;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.AutonomousCommands.StraightLineDrive;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;

public class LeftUturnMidwayGather {

        private LeftUturnMidwayGather() {
        }

        public static Command create(
                        Drivetrain drivetrain,
                        SwerveModule[] wheels,
                        IGyro gyro,
                        Shooter shooter,
                        MagazineAndFeed magazine,
                        Intake intake) {

                StraightLineDrive drive_over_ramp_to_center = new StraightLineDrive(drivetrain, 0, 187, wheels, gyro,
                                true);

                StraightLineDrive drive_through_fuel = new StraightLineDrive(drivetrain, -90, 140, wheels, gyro, true);

                StraightLineDrive drive_back_to_line_up_with_ramp = new StraightLineDrive(drivetrain, -90, -120, wheels,
                                gyro, false);

                StraightLineDrive drive_back_in_to_alliance_zone = new StraightLineDrive(drivetrain, 0, -186, wheels,
                                gyro, true);

                return Commands.runOnce(() -> gyro.setGyroOffset(90))
                                .andThen(shooter.homeHoodCommand())
                                .andThen(drive_over_ramp_to_center.alongWith(intake.intakeExtendCommand()))
                                .andThen(intake.runRollerCommand(() -> -0.6).withTimeout(0.5))
                                .andThen(drive_through_fuel.alongWith(intake.runRollerCommand(() -> -0.6)))
                                .andThen(intake.runRollerCommand(() -> 0.0).withTimeout(0.1))
                                .andThen(drive_back_to_line_up_with_ramp)
                                .andThen(drive_back_in_to_alliance_zone)
                                .andThen(drivetrain.rotateByAngleCommand(-155, gyro, true))
                                .andThen(Commands.deadline(Commands.waitSeconds(5), shooter.runShooterCommand(300, 65),
                                                Commands.sequence(Commands.waitSeconds(0.5),
                                                                Commands.startEnd(() -> magazine.setFeedVelocity(0.8),
                                                                                () -> magazine.cancel(), magazine))));
        }
}