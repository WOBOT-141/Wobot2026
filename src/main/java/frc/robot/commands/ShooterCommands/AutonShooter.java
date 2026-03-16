// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.IntakeCommands.IntakeFuel;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

public class AutonShooter extends SequentialCommandGroup {

  public AutonShooter(
      Shooter shooter,
      MagazineAndFeed magazine,
      double launchSpeed_ips,
      double hoodAngle_deg) {

    addRequirements(shooter, magazine);

    addCommands(

        // Spin shooter and move hood
        new RevShooter(launchSpeed_ips, hoodAngle_deg, shooter),

        // Wait until shooter + hood ready
        new WaitUntilCommand(() -> shooter.isReadyToShoot()),

        // Feed balls
        new IntakeFuel(magazine, 0.8),

        // Continue feeding for 5 seconds
        new WaitCommand(5),

        // Stop shooter and feeder
        Commands.runOnce(() -> {
          shooter.cancel();
          magazine.cancel();
        })

    );
  }
}
