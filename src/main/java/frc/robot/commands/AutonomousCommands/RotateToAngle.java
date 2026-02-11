// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {
  private final Drivetrain m_Drivetrain;
  private final double m_rotationAmount_deg;
  private double m_desiredBotAngle_deg;
  SlewRateLimiter m_SlewRateLimiter;

  IGyro gyro;

  /** Creates a new RotateToAngle. */

  public RotateToAngle(Drivetrain drivetrain, double rotationAmount_deg, IGyro gyro) {
    m_Drivetrain = drivetrain;
    m_rotationAmount_deg = rotationAmount_deg;
    m_SlewRateLimiter = new SlewRateLimiter(0.9);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_desiredBotAngle_deg = (gyro.getYaw() + m_rotationAmount_deg);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Rotation calculations
    double missingAngle_deg = Math.abs(gyro.getYaw() - m_desiredBotAngle_deg);
    double remainingRotation_perc = missingAngle_deg / m_rotationAmount_deg;

    // Speed calculations
    double slowSpot = 0.3;
    double m_trapizoidSpeed = 0;
    boolean rotationDone = false;

    if (missingAngle_deg == 0) {
      rotationDone = true;
    } else {
      if (remainingRotation_perc > slowSpot) {
        m_trapizoidSpeed = 1;
      } else {
        m_trapizoidSpeed = 0.1;
      }
    }

    double rotationSpeed = m_SlewRateLimiter.calculate(m_trapizoidSpeed);

    m_Drivetrain.driveSwerve(0, 0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.driveSwerve(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
