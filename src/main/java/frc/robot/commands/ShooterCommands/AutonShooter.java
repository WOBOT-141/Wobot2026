// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommands.IntakeFuel;
import frc.robot.subsystems.MagazineAndFeed;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonShooter extends SequentialCommandGroup {
  private final Shooter m_Shooter;
  private final MagazineAndFeed m_magazine;
  private final double m_launchSpeed_ips;
  private final double m_hoodAngle;

  /** Creates a new AutonShooter. */
  public AutonShooter(Shooter shooter, MagazineAndFeed magazine, double launchSpeed_ips, double hoodAngle) {
    m_Shooter = shooter;
    m_magazine = magazine;
    m_launchSpeed_ips = launchSpeed_ips;
    m_hoodAngle = hoodAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_magazine, m_Shooter);
    addCommands(
        new RevShooter(m_launchSpeed_ips, m_hoodAngle, m_Shooter)
            .alongWith(new WaitCommand(0.5)
                .andThen(new IntakeFuel(m_magazine, 0.8)))
                .withTimeout(5)
            .andThen(Commands.run(() -> m_Shooter.cancel()).alongWith(new IntakeFuel(m_magazine, 0))));

  }
}
