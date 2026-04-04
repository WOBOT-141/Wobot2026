// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateByAngle extends Command {
  private final Drivetrain m_Drivetrain;
  private double m_rotationAmount_deg;
  private double m_desiredBotAngle_deg;
  SlewRateLimiter m_SlewRateLimiter;
  double gyroyaw;
  private double remainingRotation_perc;
  private boolean m_speedFactor;

  IGyro m_gyro;

  /** Creates a new RotateToAngle. */

  public RotateByAngle(Drivetrain drivetrain, double rotationAmount_deg, IGyro gyro, boolean speedFactor) {
    m_Drivetrain = drivetrain;
    m_rotationAmount_deg = rotationAmount_deg;
    m_gyro = gyro;
    m_speedFactor = speedFactor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_desiredBotAngle_deg = m_gyro.getYaw() + m_rotationAmount_deg;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroyaw = m_gyro.getYaw();
    double missingAngle_deg = gyroyaw - m_desiredBotAngle_deg;

    SmartDashboard.putNumber("gyroyaw", gyroyaw);
    SmartDashboard.putNumber("m_desiredBotAngle_deg", m_desiredBotAngle_deg);
    SmartDashboard.putNumber("m_rotationAmount", m_rotationAmount_deg);
    SmartDashboard.putNumber("missingAngle_deg", missingAngle_deg);
    remainingRotation_perc = Math.abs(missingAngle_deg / m_rotationAmount_deg);

    // Speed calculations
    double rotationSpeed = 0.05;
    double finalRotationSpeed = 0.05;

    if (0.25 > remainingRotation_perc) {
      rotationSpeed = remainingRotation_perc * 10 - 0.15;
    } else if (0.25 <= remainingRotation_perc && remainingRotation_perc <= 0.6) {
      rotationSpeed = 0.5;
    } else if (0.6 < remainingRotation_perc && 0.95 >= remainingRotation_perc) {
      rotationSpeed = remainingRotation_perc * (-1.25) + 1.25;
    }
    if (m_speedFactor) {
      finalRotationSpeed = rotationSpeed * ((m_rotationAmount_deg) * 0.00555556);
    } else {
      finalRotationSpeed = rotationSpeed;
    }
    if (remainingRotation_perc < 0.05) {
      finalRotationSpeed = 0.05;
      if (m_rotationAmount_deg <= -1) {
        finalRotationSpeed *= -1;
      }
    }
    SmartDashboard.putNumber("m_desiredBotAngle_deg", m_desiredBotAngle_deg);
    SmartDashboard.putNumber("gyroyaw", gyroyaw);
    SmartDashboard.putNumber("missingAngle_deg", missingAngle_deg);
    SmartDashboard.putNumber("remainingRotation_perc", remainingRotation_perc);
    SmartDashboard.putNumber("finalRotationSpeed", finalRotationSpeed);

    m_Drivetrain.driveWithoutGyro(0, 0, rotationSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.driveWithoutGyro(0, 0, 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((m_desiredBotAngle_deg + 5) >= gyroyaw && (m_desiredBotAngle_deg - 5) <= gyroyaw));
    // return remainingRotation_perc > 1;
    // return false;
  }
}