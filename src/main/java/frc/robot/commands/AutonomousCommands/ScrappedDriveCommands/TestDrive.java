// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.ScrappedDriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.SwerveModule;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final double m_translationAngle;
  private final double m_distance_in;
  private final SwerveModule[] m_wheels;
  private final IGyro m_gyro;
  private double[] m_startingEncoderPos_in;
  private double m_rotationAmount_deg;
  private double m_desiredBotAngle_deg;
  double gyroyaw;
  private double remainingRotation_perc;
  private boolean m_speedFactor;

  private double distanceLeftOver_perc;
  private double finalDriveSpeed;

  /** Creates a new StraightLineDrive. */
  public TestDrive(Drivetrain drivetrain, double translationAngle, double distance, double rotationAmount_deg, SwerveModule wheels[],
      IGyro gyro, boolean speedFactor) {
    m_drivetrain = drivetrain;
    m_translationAngle = translationAngle;
    m_distance_in = distance;
    m_wheels = wheels;
    m_gyro = gyro;
    m_speedFactor = speedFactor;
    m_rotationAmount_deg = rotationAmount_deg;
    m_speedFactor = speedFactor;
    m_startingEncoderPos_in = new double[m_wheels.length];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < m_wheels.length; i++) {
      m_startingEncoderPos_in[i] = m_wheels[i].getEncoderDistance();
      SmartDashboard.putNumber("StartingEncoderPos_" + i, m_startingEncoderPos_in[i]);
    }
    finalDriveSpeed = 0.05;
    distanceLeftOver_perc = 0;
    m_desiredBotAngle_deg = m_gyro.getRawGyroAngle() + m_rotationAmount_deg;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyroyaw = m_gyro.getRawGyroAngle();
    double missingAngle_deg = gyroyaw - m_desiredBotAngle_deg;
    double[] currentEncoderPos_in = new double[m_wheels.length];
    double totalSubstractedDistance_in = 0;

    for (int i = 0; i < m_wheels.length; i++) {
      currentEncoderPos_in[i] = m_wheels[i].getEncoderDistance();
      // SmartDashboard.putNumber("CurrentEncoderPos_" + i, currentEncoderPos_in[i]);
      double encoderDifference_in = Math.abs(m_startingEncoderPos_in[i] - currentEncoderPos_in[i]);
      SmartDashboard.putNumber("Encoder difference " + i, encoderDifference_in);
      totalSubstractedDistance_in += encoderDifference_in;
    }

    double m_avgSubstractedDistance_in = totalSubstractedDistance_in / m_wheels.length;
    distanceLeftOver_perc = Math.abs(m_avgSubstractedDistance_in / m_distance_in);
    remainingRotation_perc = Math.abs(missingAngle_deg / m_rotationAmount_deg);

        double rotationSpeed = 0.05;
    double finalRotationSpeed = 0.05;

    if (0.25 > remainingRotation_perc) {
      rotationSpeed = remainingRotation_perc * 10 - 0.15;
    } else if (0.25 <= remainingRotation_perc && remainingRotation_perc <= 0.6) {
      rotationSpeed = 0.5;
    } else if (0.6 < remainingRotation_perc && 0.95 >= remainingRotation_perc) {
      rotationSpeed = (remainingRotation_perc * -1.25) + 1.25;
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

    SmartDashboard.putNumber("distanceLeftOver_perc", distanceLeftOver_perc);
    SmartDashboard.putNumber("m_AvgSubtractedDistance", m_avgSubstractedDistance_in);
    SmartDashboard.putNumber("m_distance_in", m_distance_in);

    m_drivetrain.driveWithGyro(m_translationAngle, TrapizoidCalculations(), finalRotationSpeed);
  }

  private double TrapizoidCalculations() {
    double driveSpeed = 0.05;

    if (0.10 > distanceLeftOver_perc) {
      driveSpeed = distanceLeftOver_perc * 9.5 + 0.05;
    } else if (0.10 <= distanceLeftOver_perc && distanceLeftOver_perc <= 0.8) {
      driveSpeed = 1.0;// 1.0;
    } else if (0.8 < distanceLeftOver_perc) {
      driveSpeed = distanceLeftOver_perc * (-4.52381) + 4.61905;
    } // faster

    if (m_speedFactor) {
      finalDriveSpeed = driveSpeed * ((m_distance_in) * 0.00544306);
    } else {
      finalDriveSpeed = driveSpeed * (m_distance_in);
    }

    return finalDriveSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveWithGyro(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(distanceLeftOver_perc) < 0.01 && Math.abs(m_desiredBotAngle_deg - gyroyaw) < 5;
    return Math.abs(distanceLeftOver_perc) > .99;
  }
}
