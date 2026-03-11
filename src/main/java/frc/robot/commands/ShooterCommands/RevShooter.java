// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RevShooter extends Command {
  private double m_launchSpeed_ips;
  private double m_hoodAngle_deg;
  private Shooter m_shooter; 
  /** Creates a new RevShooter. */
  public RevShooter( double launchSpeed_ips, double hoodAngle_deg, Shooter shooter) {
    m_shooter = shooter;
    m_launchSpeed_ips = launchSpeed_ips;
    m_hoodAngle_deg = hoodAngle_deg;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterVelocity(m_launchSpeed_ips);
    m_shooter.setHoodAngle(m_hoodAngle_deg);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
