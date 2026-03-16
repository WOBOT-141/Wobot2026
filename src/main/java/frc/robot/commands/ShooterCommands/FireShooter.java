// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.Feed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

public class FireShooter extends ParallelCommandGroup {

  public FireShooter(
      double shooterSpeed_ips,
      double hoodAngle_deg,
      Shooter shooter,
      Drivetrain drivetrain,
      MagazineAndFeed magazineAndFeed) {

    addRequirements(shooter, drivetrain, magazineAndFeed);

    addCommands(

        // Lock wheels immediately
        Commands.runOnce(() -> drivetrain.lockWheels(), drivetrain)

            // Start revving shooter
            .andThen(new RevShooter(shooterSpeed_ips, hoodAngle_deg, shooter))

            // Wait until shooter and hood are ready
            .alongWith(
                new WaitUntilCommand(() -> shooter.isReadyToShoot())
                    // Once ready, start feeding
                    .andThen(new Feed(magazineAndFeed))

            )

    );
  }
}
