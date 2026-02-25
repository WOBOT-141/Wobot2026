// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {
  private final Drivetrain m_Drivetrain;
  private final double m_rotationAmount_deg;
  private double m_desiredBotAngle_deg;
  SlewRateLimiter m_SlewRateLimiter;
  double gyroyaw;

  IGyro m_gyro;

  /** Creates a new RotateToAngle. */

  public RotateToAngle(Drivetrain drivetrain, double rotationAmount_deg, IGyro gyro) {
    m_Drivetrain = drivetrain;
    m_rotationAmount_deg = rotationAmount_deg;
    // m_SlewRateLimiter = new SlewRateLimiter(0.1);
    m_gyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_desiredBotAngle_deg = (m_gyro.getRealGyroAngle() + m_rotationAmount_deg) % 360;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Rotation calculations
    gyroyaw = m_gyro.getRealGyroAngle();
    double missingAngle_deg = ((gyroyaw - m_desiredBotAngle_deg) + m_rotationAmount_deg) % 360;
    double remainingRotation_perc = missingAngle_deg / m_rotationAmount_deg;

    // Speed calculations
    double slowSpot = 0.3;
    double rotationSpeed = 0.1;

    // if (m_rotationAmount_deg < 180 && m_rotationAmount_deg > -180) {
    // if (remainingRotation_perc > slowSpot) { // TODO TUNE VALUES
    // rotationSpeed = 0.2;
    // } else {
    // rotationSpeed = 0.5;
    // }
    // } else {
    // if (remainingRotation_perc > slowSpot) {
    // rotationSpeed = -0.2;
    // } else {
    // rotationSpeed = -0.5;
    // }
    // }


    // if (remainingRotation_perc > slowSpot) { // TODO TUNE VALUES
    //   rotationSpeed = 0.2;
    // } else {
    //   rotationSpeed = 0.5;
    // }

    // double rotationSpeed = m_SlewRateLimiter.calculate();
    SmartDashboard.putNumber("m_desiredBotAngle_deg", m_desiredBotAngle_deg);
    SmartDashboard.putNumber("gyroyaw", gyroyaw);
    SmartDashboard.putNumber("missingAngle_deg", missingAngle_deg);
    SmartDashboard.putNumber("remainingRotation_perc", remainingRotation_perc);

    m_Drivetrain.driveWithoutGyro(0, 0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.driveWithoutGyro(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((m_desiredBotAngle_deg + 5) >= gyroyaw && (m_desiredBotAngle_deg - 5) <= gyroyaw);
  }
}
