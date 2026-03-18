// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.ScrappedAutons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.AutonomousCommands.StraightLineDrive;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveModule;

public class Depotlease extends SequentialCommandGroup {

    public Depotlease(
            Drivetrain drivetrain,
            SwerveModule[] wheels,
            IGyro gyro,
            Intake intake,
            Shooter shooter,
            MagazineAndFeed magazine) {

        addCommands(

                Commands.parallel(Commands.sequence(
                        new StraightLineDrive(drivetrain, 90, 130, wheels, gyro, true),
                        new StraightLineDrive(drivetrain, 0, 35, wheels, gyro, true)),
                        Commands.sequence(
                                intake.intakeExtendCommand(),
                                intake.intakeSetCommand(-0.6),
                                new WaitCommand(0.5))),

                new StraightLineDrive(drivetrain, 0, 45, wheels, gyro, true),
                new StraightLineDrive(drivetrain, 0, 20, wheels, gyro, true),

                intake.intakeSetCommand(0.0),

                new StraightLineDrive(drivetrain, -75, 90, wheels, gyro, true),
                drivetrain.rotateByAngleCommand(-45, gyro, false),

                shooter.autonShootCommand(magazine, 280, 72));
    }
}