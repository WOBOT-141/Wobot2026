// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateCommandConstants;
import frc.robot.Constants.RotateVisionConstants;
import frc.robot.Util.DashboardPIDGetter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IGyro;

public class RotateByAnglePID extends Command {
  private final Drivetrain m_drive;
  private final IGyro m_gyro;
  private final PIDController m_pidController;

  private final DashboardPIDGetter m_getter;
  private double m_targetRotationAmount_deg;
  private double m_targetAngle_deg;

  public RotateByAnglePID(double rotationAmount_deg, IGyro gyro, Drivetrain drivetrain) {
    m_drive = drivetrain;
    m_gyro = gyro;
    
    m_pidController = new PIDController(RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D);
    m_getter = new DashboardPIDGetter(m_pidController, "RotateAngle", RotateVisionConstants.P, RotateVisionConstants.I, RotateVisionConstants.D , rotationAmount_deg);
    m_targetRotationAmount_deg = rotationAmount_deg;
    m_pidController.setTolerance(RotateCommandConstants.rotateTargetTolerance_deg);

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    System.out.println("RotateByAngle BEGIN");
    m_targetRotationAmount_deg = m_getter.getDashboardTarget();
    m_targetAngle_deg = m_gyro.getRawGyroAngle() + m_targetRotationAmount_deg;
    m_pidController.reset();
    m_pidController.setSetpoint(m_targetAngle_deg);
    m_getter.setPIDValuesFromDashboard();
  }

  @Override
  public void execute() {
    double currentAngle_deg = m_gyro.getRawGyroAngle();
    double pidOutput_perc = m_pidController.calculate(currentAngle_deg);
    m_drive.driveWithGyro(0, 0, pidOutput_perc);
    System.out.printf("[target:%.3f  current:%.3f  pidOut:%.3f]\n", m_targetAngle_deg, currentAngle_deg, pidOutput_perc);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stopMotors();
    System.out.println("RotateByAngle END");
  }

  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
