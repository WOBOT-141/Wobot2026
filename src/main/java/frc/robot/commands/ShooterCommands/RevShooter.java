// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command {

  private final Shooter m_shooter;
  private final double m_launchSpeed_ips;
  private final double m_hoodAngle_deg;

  public RevShooter(double launchSpeed_ips, double hoodAngle_deg, Shooter shooter) {

    m_shooter = shooter;
    m_launchSpeed_ips = launchSpeed_ips;
    m_hoodAngle_deg = hoodAngle_deg;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    // Nothing needed here
  }

  @Override
  public void execute() {

    // Send both commands repeatedly to ensure CAN packets arrive
    m_shooter.setHoodAngle(m_hoodAngle_deg);
    m_shooter.setShooterVelocity(m_launchSpeed_ips);
  }

  @Override
  public void end(boolean interrupted) {

    // Stop shooter if command ends
    m_shooter.cancel();
  }

  @Override
  public boolean isFinished() {

    // Runs until interrupted
    return false;
  }
}