// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands.ScrappedDriveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.Constants.RotateVisionConstants;
import frc.robot.Util.DashboardPIDGetter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;
import frc.robot.subsystems.SwerveModule;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final Drivetrain m_drivetrain;
  private final double m_translationAngle;
  private final double m_distance_in;
  private final SwerveModule[] m_wheels;
  private final IGyro m_gyro;
  private double[] m_startingEncoderPos_in;
  private final boolean m_speedFactor;

  private double distanceLeftOver_perc;
  private double finalDriveSpeed;

  private final PIDController m_pidController;

  private final DashboardPIDGetter m_getter;
  private double m_targetRotationAmount_deg;
  private double m_targetAngle_deg;

  /** Creates a new StraightLineDrive. */
  public DriveCommand(Drivetrain drivetrain, double translationAngle, double distance, double rotationAmount_deg, SwerveModule wheels[], IGyro gyro, boolean speedFactor) {
    m_drivetrain = drivetrain;
    m_translationAngle = translationAngle;
    m_distance_in = distance;
    m_wheels = wheels;
    m_gyro = gyro;
    m_speedFactor = speedFactor;
    
    m_startingEncoderPos_in = new double[m_wheels.length];
    m_pidController = new PIDController(RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D);
    m_getter = new DashboardPIDGetter(m_pidController, "RotateAngle", RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D , rotationAmount_deg);
    m_targetRotationAmount_deg = rotationAmount_deg;
    m_pidController.setTolerance(RotateCommandConstants.rotateTargetTolerance_deg);
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

    m_targetRotationAmount_deg = m_getter.getDashboardTarget();
    m_targetAngle_deg = m_gyro.getRawGyroAngle() + m_targetRotationAmount_deg;
    m_pidController.reset();
    m_pidController.setSetpoint(m_targetAngle_deg);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

    SmartDashboard.putNumber("distanceLeftOver_perc", distanceLeftOver_perc);
    SmartDashboard.putNumber("m_AvgSubtractedDistance", m_avgSubstractedDistance_in);
    SmartDashboard.putNumber("m_distance_in", m_distance_in);

    double currentAngle_deg = m_gyro.getRawGyroAngle();
    double pidOutput_perc = m_pidController.calculate(currentAngle_deg);

    m_drivetrain.driveWithGyro(m_translationAngle, TrapizoidCalculations(), pidOutput_perc);
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
    return (distanceLeftOver_perc >= .99 || distanceLeftOver_perc < -0) && (m_pidController.atSetpoint());
  }
}
